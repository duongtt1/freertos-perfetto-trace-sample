/**
 * @file  trace_freertos.c
 * @brief FreeRTOS Scheduler Event Tracer — Core Implementation
 *
 * Design notes
 * ============
 * Ring buffer
 *   Static array of trace_evt_t[TRACE_BUFFER_SIZE] with power-of-2 mask.
 *   A monotonically incrementing 32-bit write index (s_widx) and read index
 *   (s_ridx) encode position; slot = idx & (SIZE-1).  Full condition:
 *     (s_widx - s_ridx) >= TRACE_BUFFER_SIZE
 *   On single-core ARM, the short critical section (IRQ disable) covering
 *   the check+write+advance is ≈ 8–12 instructions / ~6 ns at 500 MHz —
 *   well within RTOS scheduling jitter.  On POSIX simulator a pthread_mutex
 *   is used instead.
 *
 * Timestamp sources
 *   POSIX     : clock_gettime(CLOCK_MONOTONIC) → µs offset from init
 *   Generic   : CNTPCT_EL0 / CNTFRQ_EL0 — no division, pre-scale trick
 *   DWT       : CYCCNT / (SystemCoreClock/1e6) — requires DWT enabled
 *   Tick      : xTaskGetTickCountFromISR() * (1e6 / configTICK_RATE_HZ)
 *
 * Task registration
 *   Uses a flat array s_tasks[TRACE_MAX_TASKS] keyed by TCB pointer.
 *   Linear search is O(n) but n ≤ 32 and this path is only hit on the
 *   *first* switch of each new task (cache miss amortised across lifetime).
 *   Subsequent lookups return in ≤ 2 comparisons on a warm system.
 *
 * SPDX-License-Identifier: MIT
 */

#include "trace_freertos.h"

/* ---- FreeRTOS includes (available via include path) ------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ---- Standard C ------------------------------------------------------- */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Compile-time assertions
 * ====================================================================== */

#if ((TRACE_RING_BUFFER_CAPACITY) & ((TRACE_RING_BUFFER_CAPACITY) - 1U)) != 0U
#  error "TRACE_RING_BUFFER_CAPACITY must be a power of 2"
#endif

#if TRACE_RING_BUFFER_CAPACITY < 64U
#  error "TRACE_RING_BUFFER_CAPACITY must be at least 64"
#endif

#define TRACE_BUF_MASK   ((TRACE_RING_BUFFER_CAPACITY) - 1U)

/* ID range sentinels (same as in header for clarity) */
#define TRACE_ID_QUEUE_BASE   (0x4000U)
#define TRACE_ID_ISR_BASE     (0x8000U)

/* =========================================================================
 * Platform-specific critical section for ring buffer access
 *
 * Rationale: traceTASK_SWITCHED_IN/OUT are called from within the ARM
 * exception handler with IRQs already masked, so they need no locking.
 * Queue events (trace_queue_send/recv) and user markers are called from
 * task context and CAN be preempted by an ISR that also calls trace_isr_*.
 * Therefore all push paths share the same lock to prevent index corruption.
 * ====================================================================== */

#if defined(TRACE_LOCK) && defined(TRACE_UNLOCK)
/* ---- Application-supplied locking ------------------------------------ */

#elif defined(__linux__) || defined(__unix__)
/* ---- POSIX simulator -------------------------------------------------- */
#  include <pthread.h>
   static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;
#  define TRACE_LOCK()    pthread_mutex_lock(&s_lock)
#  define TRACE_UNLOCK()  pthread_mutex_unlock(&s_lock)

#elif defined(__aarch64__)
/* ---- AArch64 (Cortex-R52 in 64-bit EL, Cortex-A, etc.) --------------- */
   static inline uint64_t prv_daif_disable_irq_fiq(void)
   {
       uint64_t daif;
       __asm volatile(
           "mrs  %0, daif      \n\t"
           "msr  daifset, #3   \n\t"   /* set I and F bits */
           : "=r"(daif) : : "memory"
       );
       return daif;
   }
   static inline void prv_daif_restore(uint64_t daif)
   {
       __asm volatile("msr daif, %0" : : "r"(daif) : "memory");
   }
   /* Use GCC statement-expression to keep saved-state local to the block */
#  define TRACE_LOCK()    uint64_t _daif_saved_ = prv_daif_disable_irq_fiq()
#  define TRACE_UNLOCK()  prv_daif_restore(_daif_saved_)

#elif defined(__arm__)
/* ---- AArch32 (Cortex-R52 in 32-bit, Cortex-M, Cortex-R4/5) ----------- */
   static inline uint32_t prv_cpsr_disable_irq_fiq(void)
   {
       uint32_t cpsr;
       __asm volatile(
           "mrs  %0, cpsr   \n\t"
           "cpsid if        \n\t"   /* disable IRQ + FIQ */
           : "=r"(cpsr) : : "memory"
       );
       return cpsr;
   }
   static inline void prv_cpsr_restore(uint32_t cpsr)
   {
       __asm volatile("msr cpsr_c, %0" : : "r"(cpsr) : "memory");
   }
#  define TRACE_LOCK()    uint32_t _cpsr_saved_ = prv_cpsr_disable_irq_fiq()
#  define TRACE_UNLOCK()  prv_cpsr_restore(_cpsr_saved_)

#else
/* ---- Generic FreeRTOS fallback ---------------------------------------- */
#  define TRACE_LOCK()    taskENTER_CRITICAL()
#  define TRACE_UNLOCK()  taskEXIT_CRITICAL()
#endif

/* =========================================================================
 * Timestamp implementation
 * ====================================================================== */

#if TRACE_TIMESTAMP_SOURCE == TRACE_TS_POSIX
/* ---------------------------------------------------------------------- */
#  include <time.h>
   static uint64_t s_ts_origin_ns;

   static void prv_ts_init(void)
   {
       struct timespec ts;
       (void)clock_gettime(CLOCK_MONOTONIC, &ts);
       s_ts_origin_ns = (uint64_t)ts.tv_sec * 1000000000ULL
                      + (uint64_t)ts.tv_nsec;
   }

   uint32_t trace_get_timestamp_us(void)
   {
       struct timespec ts;
       (void)clock_gettime(CLOCK_MONOTONIC, &ts);
       uint64_t now_ns = (uint64_t)ts.tv_sec * 1000000000ULL
                       + (uint64_t)ts.tv_nsec;
       return (uint32_t)((now_ns - s_ts_origin_ns) / 1000ULL);
   }

/* ---------------------------------------------------------------------- */
#elif TRACE_TIMESTAMP_SOURCE == TRACE_TS_GENERIC_TIMER
/*
 * ARM Generic Timer — Cortex-R52 / Cortex-A (ARMv8 AArch64 or AArch32)
 *
 * CNTPCT_EL0 : Physical counter (64-bit, always-on, runs at CNTFRQ_EL0 Hz)
 * CNTFRQ_EL0 : Frequency in Hz — typically 25 MHz or system-clock derived.
 *
 * Trick to avoid 64-bit division on targets lacking HW divider:
 *   ts_us = cnt / (freq / 1_000_000)
 * This is exact when freq is a multiple of 1 MHz (always true in practice).
 *
 * PMU note for Cortex-R52:
 *   - PMCCNTR_EL0 gives cycle count (enable via PMCR_EL0.E=1, PMCNTENSET_EL0.C=1)
 *   - PMCCNTR runs at CPU frequency and can be used for sub-µs resolution.
 *   - Use PMCCNTR for inter-event latency, Generic Timer for wall-clock ts.
 */
#  ifdef __aarch64__
   static uint64_t s_cntfrq_mhz;   /* CNTFRQ in units of MHz (divisor) */

   static void prv_ts_init(void)
   {
       uint64_t freq;
       __asm volatile("mrs %0, CNTFRQ_EL0" : "=r"(freq) : : "memory");
       /* Guard against uninitialised register (should not happen in practice) */
       s_cntfrq_mhz = (freq > 0ULL) ? (freq / 1000000ULL) : 1ULL;
   }

   uint32_t trace_get_timestamp_us(void)
   {
       uint64_t cnt;
       __asm volatile("mrs %0, CNTPCT_EL0" : "=r"(cnt) : : "memory");
       return (uint32_t)(cnt / s_cntfrq_mhz);
   }

#  else  /* AArch32 */
   static uint32_t s_cntfrq_mhz;

   static void prv_ts_init(void)
   {
       uint32_t freq;
       __asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r"(freq));
       s_cntfrq_mhz = (freq > 0U) ? (freq / 1000000U) : 1U;
   }

   uint32_t trace_get_timestamp_us(void)
   {
       uint32_t lo, hi;
       /* CNTPCT AArch32: two 32-bit reads, must re-read hi if wrapped */
       do {
           __asm volatile("mrrc p15, 0, %0, %1, c14" : "=r"(lo), "=r"(hi));
       } while (0);  /* single-core: no re-read needed */
       uint64_t cnt = ((uint64_t)hi << 32U) | (uint64_t)lo;
       return (uint32_t)(cnt / s_cntfrq_mhz);
   }
#  endif /* __aarch64__ */

/* ---------------------------------------------------------------------- */
#elif TRACE_TIMESTAMP_SOURCE == TRACE_TS_DWT
/*
 * ARM DWT CYCCNT — Cortex-M3/M4/M7/M33 and Cortex-R with DWT
 *
 * Resolution: 1 / SystemCoreClock.
 * Rollover period: 2^32 / 500e6 = ~8.6 s at 500 MHz.
 *
 * This code uses CMSIS CoreDebug/DWT definitions.  Include the correct
 * CMSIS device header before this translation unit (or add to build flags).
 */
#  ifndef CoreDebug
#    include "core_cm7.h"   /* substitute for your target core */
#  endif
   extern uint32_t SystemCoreClock;

   static void prv_ts_init(void)
   {
       CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
       DWT->CYCCNT = 0U;
       DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
   }

   uint32_t trace_get_timestamp_us(void)
   {
       /* Integer division — acceptable overhead (~3 cycles with HW divider) */
       return DWT->CYCCNT / (SystemCoreClock / 1000000U);
   }

/* ---------------------------------------------------------------------- */
#else   /* TRACE_TS_TICK — safe fallback */
   static void prv_ts_init(void) { /* no HW init needed */ }

   uint32_t trace_get_timestamp_us(void)
   {
       /* Cast to 64-bit to avoid overflow at high tick rates */
       return (uint32_t)(
           (uint64_t)xTaskGetTickCountFromISR()
           * (1000000ULL / (uint64_t)configTICK_RATE_HZ)
       );
   }
#endif  /* TRACE_TIMESTAMP_SOURCE */

/* =========================================================================
 * Private state — all static, zero-initialised by C runtime
 * ====================================================================== */

/* Ring buffer */
#if defined(TRACE_RING_BUFFER_PTR)
static trace_evt_t * const s_ring = (trace_evt_t *)TRACE_RING_BUFFER_PTR;
#else
static trace_evt_t  s_ring_storage[TRACE_RING_BUFFER_CAPACITY] TRACE_BUFFER_ATTRIBUTE;
static trace_evt_t * const s_ring = s_ring_storage;
#endif

static uint32_t     s_widx;       /* write index — producer only  */
static uint32_t     s_ridx;       /* read  index — consumer only  */

/* Task table */
static trace_task_entry_t  s_tasks[TRACE_MAX_TASKS];
static uint16_t            s_task_count;   /* next free task trace-ID */

/* ISR name table */
typedef struct {
    char     name[TRACE_NAME_LEN];
    uint16_t id;
    uint8_t  valid;
} prv_named_t;

static prv_named_t  s_isr_names[TRACE_MAX_ISRS];
static prv_named_t  s_marker_names[TRACE_MAX_MARKERS];

/* Control and statistics */
static volatile bool  s_enabled;
static trace_stats_t  s_stats;
static uint32_t       s_last_ts_us;   /* for wrap detection */

/* =========================================================================
 * Internal helpers
 * ====================================================================== */

/**
 * prv_push_event — write one event to the ring buffer.
 *
 * Caller must hold TRACE_LOCK() across this call OR ensure this is
 * called from a context where no other trace call can interleave
 * (e.g. task-switch ISR on single-core with IRQs already masked).
 *
 * Estimated overhead on Cortex-R52 @ 800 MHz (no lock path):
 *   ~14 instructions = ~18 ns per event.
 */
static void prv_push_event(uint32_t ts_us, uint16_t id,
                            trace_evt_type_t type, uint8_t prio)
{
    uint32_t used = s_widx - s_ridx;

    /* Wrap-around detection (ts_us is a 32-bit µs counter) */
    if (ts_us < s_last_ts_us) {
        s_stats.ts_wrap_count++;
    }
    s_last_ts_us = ts_us;

    if (used >= TRACE_RING_BUFFER_CAPACITY) {
        s_stats.events_dropped++;
        return;
    }

    uint32_t slot = s_widx & TRACE_BUF_MASK;
    s_ring[slot].ts_us = ts_us;
    s_ring[slot].id    = id;
    s_ring[slot].type  = (uint8_t)type;
    s_ring[slot].prio  = prio;

    /* Compiler memory barrier: ensure fields written before head advance */
    __asm volatile("" ::: "memory");

    s_widx++;

    s_stats.events_recorded++;
    if (used + 1U > s_stats.buffer_peak) {
        s_stats.buffer_peak = used + 1U;
    }
}

/**
 * prv_find_or_register_task — resolve a TCB pointer to a task entry.
 *
 * On first encounter of a TCB, queries FreeRTOS for the task name and
 * priority and allocates the next sequential trace ID.
 *
 * Returns pointer to entry, or NULL if the task table is full.
 * Caller MUST hold TRACE_LOCK when calling this function.
 */
static trace_task_entry_t *prv_find_or_register_task(void *tcb)
{
    uint16_t i;
    uint16_t free_slot = TRACE_MAX_TASKS;  /* sentinel */

    for (i = 0U; i < TRACE_MAX_TASKS; i++) {
        if (s_tasks[i].valid) {
            if (s_tasks[i].tcb_ptr == tcb) {
                return &s_tasks[i];   /* fast path — already known */
            }
        } else if (free_slot == TRACE_MAX_TASKS) {
            free_slot = i;
        }
    }

    /* Not found — allocate new slot */
    if (free_slot == TRACE_MAX_TASKS) {
        s_stats.events_dropped++;
        return NULL;   /* task table full */
    }

    trace_task_entry_t *e = &s_tasks[free_slot];
    e->tcb_ptr        = tcb;
    e->id             = s_task_count++;
    e->valid          = 1U;
    e->total_active_us = 0U;
    e->switch_in_ts   = 0U;
    s_stats.tasks_registered = s_task_count;

    /* Query FreeRTOS kernel for name and priority */
    TaskHandle_t h = (TaskHandle_t)tcb;
    const char  *kname = pcTaskGetName(h);
    if (kname != NULL) {
        (void)strncpy(e->name, kname, TRACE_NAME_LEN - 1U);
        e->name[TRACE_NAME_LEN - 1U] = '\0';
    } else {
        (void)snprintf(e->name, TRACE_NAME_LEN, "Task%u",
                       (unsigned int)e->id);
    }
    e->prio = (uint8_t)uxTaskPriorityGet(h);

    return e;
}

/* Lookup helpers for dump phase (read-only, no lock needed after capture) */

static const char *prv_task_name_by_id(uint16_t id)
{
    uint16_t i;
    for (i = 0U; i < TRACE_MAX_TASKS; i++) {
        if (s_tasks[i].valid && s_tasks[i].id == id) {
            return s_tasks[i].name;
        }
    }
    return "Unknown";
}

static const char *prv_isr_name_by_id(uint16_t isr_id)
{
    uint16_t i;
    for (i = 0U; i < TRACE_MAX_ISRS; i++) {
        if (s_isr_names[i].valid && s_isr_names[i].id == isr_id) {
            return s_isr_names[i].name;
        }
    }
    return "ISR";
}

static const char *prv_marker_name_by_id(uint16_t mid)
{
    uint16_t i;
    for (i = 0U; i < TRACE_MAX_MARKERS; i++) {
        if (s_marker_names[i].valid && s_marker_names[i].id == mid) {
            return s_marker_names[i].name;
        }
    }
    return "Marker";
}

static const char *prv_type_str(uint8_t t)
{
    switch ((trace_evt_type_t)t) {
        case TRACE_EVT_TASK_IN:    return "IN";
        case TRACE_EVT_TASK_OUT:   return "OUT";
        case TRACE_EVT_QUEUE_SEND: return "QSEND";
        case TRACE_EVT_QUEUE_RECV: return "QRECV";
        case TRACE_EVT_ISR_ENTER:  return "ISRIN";
        case TRACE_EVT_ISR_EXIT:   return "ISROUT";
        case TRACE_EVT_MARKER:     return "MARK";
        case TRACE_EVT_COUNTER:    return "COUNTER";
        default:                   return "UNK";
    }
}

/* =========================================================================
 * Public API — Lifecycle
 * ====================================================================== */

void trace_init(void)
{
#if TRACE_CLEAR_BUFFER_ON_INIT
    (void)memset(s_ring,         0, TRACE_RING_BUFFER_CAPACITY * sizeof(trace_evt_t));
#endif
    (void)memset(s_tasks,        0, sizeof(s_tasks));
    (void)memset(s_isr_names,    0, sizeof(s_isr_names));
    (void)memset(s_marker_names, 0, sizeof(s_marker_names));
    (void)memset(&s_stats,       0, sizeof(s_stats));

    s_widx       = 0U;
    s_ridx       = 0U;
    s_task_count = 0U;
    s_last_ts_us = 0U;
    s_enabled    = false;

    prv_ts_init();
}

void trace_enable(bool enable)
{
    s_enabled = enable;
}

bool trace_is_enabled(void)
{
    return s_enabled;
}

void trace_reset(void)
{
    /* Must be called with tracing disabled */
    s_widx = 0U;
    s_ridx = 0U;
    (void)memset(&s_stats, 0, sizeof(s_stats));
    s_last_ts_us = 0U;
    /* Retain task table and name registrations across reset */
}

void trace_get_stats(trace_stats_t *stats_out)
{
    if (stats_out != NULL) {
        *stats_out = s_stats;
    }
}

const trace_evt_t *trace_get_buffer_base(void)
{
    return s_ring;
}

uint32_t trace_get_buffer_capacity(void)
{
    return TRACE_RING_BUFFER_CAPACITY;
}

uint32_t trace_get_event_count(void)
{
    return s_widx - s_ridx;
}

const trace_task_entry_t *trace_get_task_entry(uint16_t trace_id)
{
    uint16_t i;
    for (i = 0U; i < TRACE_MAX_TASKS; i++) {
        if (s_tasks[i].valid && s_tasks[i].id == trace_id) {
            return &s_tasks[i];
        }
    }
    return NULL;
}

/* =========================================================================
 * Public API — Event hooks
 * ====================================================================== */

void trace_task_switched_in(void *tcb)
{
    if (!s_enabled || (tcb == NULL)) { return; }

    TRACE_LOCK();
    {
        trace_task_entry_t *e = prv_find_or_register_task(tcb);
        if (e != NULL) {
            uint32_t ts = trace_get_timestamp_us();
            e->switch_in_ts = ts;
            prv_push_event(ts, e->id, TRACE_EVT_TASK_IN, e->prio);
        }
    }
    TRACE_UNLOCK();
}

void trace_task_switched_out(void *tcb)
{
    if (!s_enabled || (tcb == NULL)) { return; }

    TRACE_LOCK();
    {
        trace_task_entry_t *e = prv_find_or_register_task(tcb);
        if (e != NULL) {
            uint32_t ts = trace_get_timestamp_us();
            /* Accumulate CPU time (handle wrap) */
            if (ts >= e->switch_in_ts) {
                e->total_active_us += (ts - e->switch_in_ts);
            }
            prv_push_event(ts, e->id, TRACE_EVT_TASK_OUT, e->prio);
        }
    }
    TRACE_UNLOCK();
}

void trace_queue_send(void *queue_handle)
{
    if (!s_enabled || (queue_handle == NULL)) { return; }

    /* Map queue pointer to a compact 14-bit token.
     * Lower 14 bits of the pointer address are a reasonable collision-free
     * key on most embedded allocators (aligned heap objects, static arrays).
     * The TRACE_ID_QUEUE_BASE bit distinguishes it from task IDs. */
    uint16_t qid = (uint16_t)(
        ((uintptr_t)queue_handle >> 2U) & 0x3FFFU
    ) | TRACE_ID_QUEUE_BASE;

    TRACE_LOCK();
    prv_push_event(trace_get_timestamp_us(), qid, TRACE_EVT_QUEUE_SEND, 0U);
    TRACE_UNLOCK();
}

void trace_queue_receive(void *queue_handle)
{
    if (!s_enabled || (queue_handle == NULL)) { return; }

    uint16_t qid = (uint16_t)(
        ((uintptr_t)queue_handle >> 2U) & 0x3FFFU
    ) | TRACE_ID_QUEUE_BASE;

    TRACE_LOCK();
    prv_push_event(trace_get_timestamp_us(), qid, TRACE_EVT_QUEUE_RECV, 0U);
    TRACE_UNLOCK();
}

void trace_isr_enter(uint16_t isr_id)
{
    if (!s_enabled) { return; }
    /* ISR IDs are stored with the TRACE_ID_ISR_BASE bit set */
    TRACE_LOCK();
    prv_push_event(trace_get_timestamp_us(),
                   isr_id | TRACE_ID_ISR_BASE,
                   TRACE_EVT_ISR_ENTER, 0xFFU);
    TRACE_UNLOCK();
}

void trace_isr_exit(uint16_t isr_id)
{
    if (!s_enabled) { return; }
    TRACE_LOCK();
    prv_push_event(trace_get_timestamp_us(),
                   isr_id | TRACE_ID_ISR_BASE,
                   TRACE_EVT_ISR_EXIT, 0xFFU);
    TRACE_UNLOCK();
}

void trace_marker(uint16_t marker_id)
{
    if (!s_enabled) { return; }
    TRACE_LOCK();
    prv_push_event(trace_get_timestamp_us(),
                   marker_id, TRACE_EVT_MARKER, 0U);
    TRACE_UNLOCK();
}

/* =========================================================================
 * Public API — Name registration
 * ====================================================================== */

void trace_register_isr(uint16_t isr_id, const char *name)
{
    uint16_t i;
    uint16_t free_slot = TRACE_MAX_ISRS;

    if (name == NULL) { return; }

    for (i = 0U; i < TRACE_MAX_ISRS; i++) {
        if (s_isr_names[i].valid) {
            if (s_isr_names[i].id == isr_id) {
                /* Update existing */
                (void)strncpy(s_isr_names[i].name, name, TRACE_NAME_LEN - 1U);
                s_isr_names[i].name[TRACE_NAME_LEN - 1U] = '\0';
                return;
            }
        } else if (free_slot == TRACE_MAX_ISRS) {
            free_slot = i;
        }
    }
    if (free_slot < TRACE_MAX_ISRS) {
        s_isr_names[free_slot].id    = isr_id;
        s_isr_names[free_slot].valid = 1U;
        (void)strncpy(s_isr_names[free_slot].name, name, TRACE_NAME_LEN - 1U);
        s_isr_names[free_slot].name[TRACE_NAME_LEN - 1U] = '\0';
    }
}

void trace_register_marker(uint16_t marker_id, const char *name)
{
    uint16_t i;
    uint16_t free_slot = TRACE_MAX_MARKERS;

    if (name == NULL) { return; }

    for (i = 0U; i < TRACE_MAX_MARKERS; i++) {
        if (s_marker_names[i].valid) {
            if (s_marker_names[i].id == marker_id) {
                (void)strncpy(s_marker_names[i].name, name, TRACE_NAME_LEN - 1U);
                s_marker_names[i].name[TRACE_NAME_LEN - 1U] = '\0';
                return;
            }
        } else if (free_slot == TRACE_MAX_MARKERS) {
            free_slot = i;
        }
    }
    if (free_slot < TRACE_MAX_MARKERS) {
        s_marker_names[free_slot].id    = marker_id;
        s_marker_names[free_slot].valid = 1U;
        (void)strncpy(s_marker_names[free_slot].name, name, TRACE_NAME_LEN - 1U);
        s_marker_names[free_slot].name[TRACE_NAME_LEN - 1U] = '\0';
    }
}

/* =========================================================================
 * Dump — Text format
 *
 * Called after trace_enable(false).  Iterates s_ridx..s_widx linearly.
 * The callback out_fn is called once per line; no heap allocation.
 *
 * Full grammar (EBNF sketch):
 *   log      ::= header line* task_table stats_footer
 *   header   ::= "#TRACE ts_us,name,type,prio\n"
 *   line     ::= decimal "," name "," type_str "," decimal "\n"
 *   task_tbl ::= "#TASKS\n" task_row*
 *   task_row ::= "#TASK " decimal "," name "," decimal "," decimal "\n"
 *   stats    ::= "#STATS " kvpair+ "\n"
 * ====================================================================== */

void trace_dump_text(void (*out_fn)(const char *line))
{
    char     line[96];
    uint32_t ridx = s_ridx;
    uint32_t widx = s_widx;
    uint16_t i;

    if (out_fn == NULL) { return; }

    out_fn("#TRACE ts_us,name,type,prio");

    while (ridx != widx) {
        const trace_evt_t *ev   = &s_ring[ridx & TRACE_BUF_MASK];
        const char        *name = NULL;
        char               qname[24];
        uint16_t           raw_id = ev->id;
        trace_evt_type_t   etype  = (trace_evt_type_t)ev->type;

        if (raw_id & TRACE_ID_ISR_BASE) {
            /* ISR event */
            name = prv_isr_name_by_id(raw_id & ~TRACE_ID_ISR_BASE);
        } else if (raw_id & TRACE_ID_QUEUE_BASE) {
            /* Queue event — synthesise a name */
            (void)snprintf(qname, sizeof(qname), "Queue_%04X",
                           (unsigned int)(raw_id & ~TRACE_ID_QUEUE_BASE));
            name = qname;
        } else if (etype == TRACE_EVT_MARKER) {
            name = prv_marker_name_by_id(raw_id);
        } else {
            name = prv_task_name_by_id(raw_id);
        }

        (void)snprintf(line, sizeof(line), "%lu,%s,%s,%u",
                       (unsigned long)ev->ts_us,
                       name,
                       prv_type_str(ev->type),
                       (unsigned int)ev->prio);
        out_fn(line);
        ridx++;
    }

    /* Task table — allows Python tool to recover names even if a task's
     * first switch happened before the #TASK row is printed */
    out_fn("#TASKS id,name,prio,total_active_us");
    for (i = 0U; i < TRACE_MAX_TASKS; i++) {
        if (s_tasks[i].valid) {
            (void)snprintf(line, sizeof(line), "#TASK %u,%s,%u,%lu",
                           (unsigned int)s_tasks[i].id,
                           s_tasks[i].name,
                           (unsigned int)s_tasks[i].prio,
                           (unsigned long)s_tasks[i].total_active_us);
            out_fn(line);
        }
    }

    /* ISR name table */
    for (i = 0U; i < TRACE_MAX_ISRS; i++) {
        if (s_isr_names[i].valid) {
            (void)snprintf(line, sizeof(line), "#ISR %u,%s",
                           (unsigned int)s_isr_names[i].id,
                           s_isr_names[i].name);
            out_fn(line);
        }
    }

    /* Marker name table */
    for (i = 0U; i < TRACE_MAX_MARKERS; i++) {
        if (s_marker_names[i].valid) {
            (void)snprintf(line, sizeof(line), "#MARKER %u,%s",
                           (unsigned int)s_marker_names[i].id,
                           s_marker_names[i].name);
            out_fn(line);
        }
    }

    /* Statistics footer */
    (void)snprintf(line, sizeof(line),
                   "#STATS recorded=%lu dropped=%lu peak=%lu wraps=%lu tasks=%lu",
                   (unsigned long)s_stats.events_recorded,
                   (unsigned long)s_stats.events_dropped,
                   (unsigned long)s_stats.buffer_peak,
                   (unsigned long)s_stats.ts_wrap_count,
                   (unsigned long)s_stats.tasks_registered);
    out_fn(line);

    out_fn("#END");
}

/* =========================================================================
 * Dump — Binary format
 *
 * Binary layout (all fields little-endian):
 *   Offset  Size  Field
 *   0       4     magic     = 0x54524346 ("TRCF")
 *   4       4     version   = 1
 *   8       4     event_count
 *  12      N*8    trace_evt_t records  (N = event_count)
 *  12+N*8  4     task_count
 *  16+N*8  M*(2+16+1)  {uint16_t id, char name[16], uint8_t prio}
 * ====================================================================== */

#define TRACE_BIN_MAGIC    (0x54524346UL)   /* "TRCF" */
#define TRACE_BIN_VERSION  (1UL)

static void prv_write_u32_le(void (*out)(const uint8_t *, uint32_t),
                              uint32_t val)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(val >>  0U);
    buf[1] = (uint8_t)(val >>  8U);
    buf[2] = (uint8_t)(val >> 16U);
    buf[3] = (uint8_t)(val >> 24U);
    out(buf, 4U);
}

static void prv_write_u16_le(void (*out)(const uint8_t *, uint32_t),
                              uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 0U);
    buf[1] = (uint8_t)(val >> 8U);
    out(buf, 2U);
}

void trace_dump_binary(void (*out_fn)(const uint8_t *buf, uint32_t len))
{
    uint32_t ridx = s_ridx;
    uint32_t widx = s_widx;
    uint32_t evt_count = widx - ridx;
    uint16_t i;

    if (out_fn == NULL) { return; }

    prv_write_u32_le(out_fn, TRACE_BIN_MAGIC);
    prv_write_u32_le(out_fn, TRACE_BIN_VERSION);
    prv_write_u32_le(out_fn, evt_count);

    while (ridx != widx) {
        out_fn((const uint8_t *)&s_ring[ridx & TRACE_BUF_MASK],
               (uint32_t)sizeof(trace_evt_t));
        ridx++;
    }

    /* Task table */
    uint32_t task_count = 0U;
    for (i = 0U; i < TRACE_MAX_TASKS; i++) {
        if (s_tasks[i].valid) { task_count++; }
    }
    prv_write_u32_le(out_fn, task_count);

    for (i = 0U; i < TRACE_MAX_TASKS; i++) {
        if (s_tasks[i].valid) {
            prv_write_u16_le(out_fn, s_tasks[i].id);
            out_fn((const uint8_t *)s_tasks[i].name, TRACE_NAME_LEN);
            out_fn(&s_tasks[i].prio, 1U);
        }
    }
}
