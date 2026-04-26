/**
 * @file  trace_freertos.h
 * @brief FreeRTOS Scheduler Event Tracer — Public API
 *
 * Hooks into FreeRTOS trace macros to capture task-switch, queue,
 * ISR, and user-marker events into a static ring buffer with near-zero
 * runtime overhead. Designed primarily for Cortex-R52 (ARMv8-R) with
 * Generic Timer, DWT, or POSIX clock as timestamp sources.
 *
 * Dump flow:
 *   FreeRTOS kernel
 *     └─ traceTASK_SWITCHED_IN / traceTASK_SWITCHED_OUT
 *          └─ trace_freertos.c  →  ring buffer  →  uart_dump.c  →  log.txt
 *                                                                       │
 *                                                              convert_perfetto.py
 *                                                                       │
 *                                                              trace.json → Perfetto UI
 *
 * SPDX-License-Identifier: MIT
 * Target: C99, MISRA-C:2012 advisory
 */

#ifndef TRACE_FREERTOS_H
#define TRACE_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Build-time configuration  (override with -DTRACE_xxx=value)
 * ====================================================================== */

/**
 * Ring buffer depth (number of trace_evt_t slots).
 * MUST be a power of 2. At 8 bytes per event, 4096 = 32 KB SRAM.
 */
#ifndef TRACE_BUFFER_SIZE
#  define TRACE_BUFFER_SIZE     (4096U)
#endif

/** Maximum character length of a task/ISR name (includes NUL terminator). */
#ifndef TRACE_NAME_LEN
#  define TRACE_NAME_LEN        (24U)
#endif

/** Maximum number of unique tasks that can be registered at runtime. */
#ifndef TRACE_MAX_TASKS
#  define TRACE_MAX_TASKS       (64U)
#endif

/** Maximum number of named ISR slots. */
#ifndef TRACE_MAX_ISRS
#  define TRACE_MAX_ISRS        (64U)
#endif

/** Maximum number of named user-marker slots. */
#ifndef TRACE_MAX_MARKERS
#  define TRACE_MAX_MARKERS     (128U)
#endif

/**
 * Optional placement/storage control for the event ring buffer.
 *
 * Default:
 *   - Storage is allocated internally inside trace_freertos.c
 *   - Capacity is TRACE_BUFFER_SIZE events
 *
 * Custom RAM placement examples:
 *   - Put internal storage in a linker section:
 *       #define TRACE_BUFFER_ATTRIBUTE __attribute__((section(".trace_buf")))
 *
 *   - Use an application-owned RAM region:
 *       #define TRACE_RING_BUFFER_PTR ((trace_evt_t *)0x68000000U)
 *       #define TRACE_RING_BUFFER_CAPACITY (524288U)
 *
 * When TRACE_RING_BUFFER_PTR is supplied, trace_freertos.c does not allocate
 * its own event array. The pointed memory must be writable and large enough
 * for TRACE_RING_BUFFER_CAPACITY entries.
 */
#ifndef TRACE_BUFFER_ATTRIBUTE
#  define TRACE_BUFFER_ATTRIBUTE
#endif

#ifndef TRACE_RING_BUFFER_CAPACITY
#  define TRACE_RING_BUFFER_CAPACITY TRACE_BUFFER_SIZE
#endif

#ifndef TRACE_CLEAR_BUFFER_ON_INIT
#  define TRACE_CLEAR_BUFFER_ON_INIT (1)
#endif

/**
 * Timestamp source selector.
 *
 * TRACE_TS_GENERIC_TIMER  — ARM Generic Timer (CNTPCT_EL0 / CNTFRQ_EL0).
 *                           Best for Cortex-R52, Cortex-A. Requires firmware
 *                           to initialise CNTFRQ_EL0 before trace_init().
 *
 * TRACE_TS_DWT            — ARM DWT CYCCNT (Cortex-M / Cortex-R with DWT).
 *                           Call trace_ts_dwt_init() or ensure DWT enabled
 *                           before trace_init().
 *
 * TRACE_TS_POSIX          — clock_gettime(CLOCK_MONOTONIC). Linux / POSIX
 *                           FreeRTOS simulator. DEFAULT when not on ARM.
 *
 * TRACE_TS_TICK           — FreeRTOS tick counter. Lowest resolution
 *                           (configTICK_RATE_HZ). Safe absolute fallback.
 */
#define TRACE_TS_GENERIC_TIMER  (0)
#define TRACE_TS_DWT            (1)
#define TRACE_TS_POSIX          (2)
#define TRACE_TS_TICK           (3)

#ifndef TRACE_TIMESTAMP_SOURCE
#  if defined(__linux__) || defined(__unix__) || defined(_POSIX_VERSION)
#    define TRACE_TIMESTAMP_SOURCE  TRACE_TS_POSIX
#  elif defined(TRACE_USE_GENERIC_TIMER)
#    define TRACE_TIMESTAMP_SOURCE  TRACE_TS_GENERIC_TIMER
#  elif defined(TRACE_USE_DWT)
#    define TRACE_TIMESTAMP_SOURCE  TRACE_TS_DWT
#  else
#    define TRACE_TIMESTAMP_SOURCE  TRACE_TS_TICK
#  endif
#endif

/* =========================================================================
 * Event type enumeration
 * ====================================================================== */

/**
 * @brief Identifies what kind of scheduler or application event was captured.
 *
 * Stored in the 8-bit `type` field of trace_evt_t.
 * Values 0x00-0x07 are reserved for the trace engine.
 * Values 0x08-0xFE are available for user-defined counter events.
 */
typedef enum {
    TRACE_EVT_TASK_IN    = 0x00U,  /**< Task switched in (becomes running)   */
    TRACE_EVT_TASK_OUT   = 0x01U,  /**< Task switched out (preempted/blocked) */
    TRACE_EVT_QUEUE_SEND = 0x02U,  /**< Queue / semaphore / mutex send/give   */
    TRACE_EVT_QUEUE_RECV = 0x03U,  /**< Queue / semaphore / mutex recv/take   */
    TRACE_EVT_ISR_ENTER  = 0x04U,  /**< ISR entry (user-called)               */
    TRACE_EVT_ISR_EXIT   = 0x05U,  /**< ISR exit  (user-called)               */
    TRACE_EVT_MARKER     = 0x06U,  /**< Instant user marker                   */
    TRACE_EVT_COUNTER    = 0x07U,  /**< Generic counter value snapshot        */
    TRACE_EVT_INVALID    = 0xFFU   /**< Sentinel / uninitialized slot         */
} trace_evt_type_t;

/* =========================================================================
 * Core event struct — 8 bytes, no padding, cache-line friendly
 * ====================================================================== */

/**
 * @brief Single trace event as stored in the ring buffer.
 *
 * All fields are packed into exactly 8 bytes to maximize ring-buffer
 * density and keep push overhead below 10 instructions on Cortex-R52.
 *
 * `id` encoding:
 *   0x0000 – 0x3FFF  →  task trace ID (assigned sequentially)
 *   0x4000 – 0x7FFF  →  queue handle index (lower 14 bits of handle)
 *   0x8000 – 0xFFFF  →  ISR ID (user-supplied, upper bit set)
 */
typedef struct {
    uint32_t ts_us;   /**< Timestamp in microseconds since trace_init()      */
    uint16_t id;      /**< Entity ID: task / queue / ISR (see encoding above) */
    uint8_t  type;    /**< One of trace_evt_type_t                           */
    uint8_t  prio;    /**< Task priority [0-255] or 0xFF for ISR events      */
} trace_evt_t;

/* Compile-time layout assertion (C11 / C99 + GCC extension) */
#ifdef __STDC_VERSION__
#  if __STDC_VERSION__ >= 201112L
     _Static_assert(sizeof(trace_evt_t) == 8U,
                    "trace_evt_t must be exactly 8 bytes — check padding");
#  endif
#endif

/* =========================================================================
 * Task registration record (internal, exposed for unit-test access)
 * ====================================================================== */

typedef struct {
    void    *tcb_ptr;               /**< FreeRTOS TCB pointer (opaque key)  */
    char     name[TRACE_NAME_LEN];  /**< Task name (NUL-terminated)         */
    uint32_t total_active_us;       /**< Cumulative running time (µs)       */
    uint32_t switch_in_ts;          /**< Timestamp of last switch-in        */
    uint16_t id;                    /**< Assigned monotonic trace ID        */
    uint8_t  prio;                  /**< FreeRTOS priority                  */
    uint8_t  valid;                 /**< Non-zero if slot is occupied       */
} trace_task_entry_t;

/* =========================================================================
 * Runtime statistics
 * ====================================================================== */

typedef struct {
    uint32_t events_recorded;   /**< Total events successfully written     */
    uint32_t events_dropped;    /**< Events lost because buffer was full   */
    uint32_t buffer_peak;       /**< Highest observed fill level (events)  */
    uint32_t ts_wrap_count;     /**< Times 32-bit µs timestamp wrapped     */
    uint32_t tasks_registered;  /**< Unique tasks seen during capture      */
} trace_stats_t;

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief Initialise the trace subsystem.
 *
 * Clears the ring buffer, resets statistics, initialises the timestamp
 * source, and sets up internal tables.  Must be called **before**
 * vTaskStartScheduler() and before any trace_register_xxx() calls.
 *
 * Tracing is DISABLED after init; call trace_enable(true) to start.
 */
void trace_init(void);

/**
 * @brief Enable or disable event capture.
 *
 * Thread-safe: may be called from task or interrupt context.
 * Disabling does not flush or reset the buffer.
 *
 * @param enable  true → capture events, false → silently discard
 */
void trace_enable(bool enable);

/** @brief Return current enable state. */
bool trace_is_enabled(void);

/**
 * @brief Get current timestamp in microseconds.
 *
 * Selects the hardware/SW source configured at compile time.
 * Exposed so application code can correlate its own timestamps.
 */
uint32_t trace_get_timestamp_us(void);

/* -----------------------------------------------------------------------
 * Hooks called from FreeRTOS trace macros in FreeRTOSConfig.h
 * These are NOT meant to be called directly by application code.
 * ----------------------------------------------------------------------- */

/** Called by traceTASK_SWITCHED_IN() with pxCurrentTCB. */
void trace_task_switched_in(void *tcb);

/** Called by traceTASK_SWITCHED_OUT() with pxCurrentTCB. */
void trace_task_switched_out(void *tcb);

/** Called by traceQUEUE_SEND() with the queue handle. */
void trace_queue_send(void *queue_handle);

/** Called by traceQUEUE_RECEIVE() with the queue handle. */
void trace_queue_receive(void *queue_handle);

/* -----------------------------------------------------------------------
 * Optional ISR tracing (user-called from within ISR body)
 * ----------------------------------------------------------------------- */

/**
 * @brief Record ISR entry.
 *
 * @param isr_id  Application-defined ID in range [0, 0x7FFF].
 *                Use trace_register_isr() to associate a name.
 *
 * @note  Place at the very start of the ISR body, before any processing.
 *        Do NOT call this from within a FreeRTOS portYIELD_FROM_ISR() region.
 */
void trace_isr_enter(uint16_t isr_id);

/**
 * @brief Record ISR exit.
 *
 * @param isr_id  Same value passed to trace_isr_enter().
 *
 * @note  Place as the very last statement before `return`.
 */
void trace_isr_exit(uint16_t isr_id);

/* -----------------------------------------------------------------------
 * User markers
 * ----------------------------------------------------------------------- */

/**
 * @brief Insert an instant marker event.
 *
 * Renders as a vertical line in Perfetto.
 *
 * @param marker_id  Application-defined ID.  Use trace_register_marker()
 *                   for a human-readable name in the output.
 */
void trace_marker(uint16_t marker_id);

/* -----------------------------------------------------------------------
 * Name registration
 * ----------------------------------------------------------------------- */

/**
 * @brief Associate a human-readable name with an ISR ID.
 *
 * @param isr_id  Must match the isr_id passed to trace_isr_enter/exit.
 * @param name    Pointer to a string literal or a buffer with static lifetime.
 */
void trace_register_isr(uint16_t isr_id, const char *name);

/**
 * @brief Associate a human-readable name with a marker ID.
 */
void trace_register_marker(uint16_t marker_id, const char *name);

/* -----------------------------------------------------------------------
 * Dump (call after trace_enable(false))
 * ----------------------------------------------------------------------- */

/**
 * @brief Dump all buffered events in human-readable text format.
 *
 * Output format (one event per line):
 *   <ts_us>,<name>,<type_str>,<prio>
 *
 * Metadata lines prefixed with '#' carry task table and statistics:
 *   #TRACE  — header sentinel
 *   #TASK   id,name,prio,total_active_us
 *   #STATS  recorded=N dropped=N peak=N wraps=N
 *
 * @param out_fn  Callback receiving NUL-terminated lines.
 *                Will be called once per event plus metadata lines.
 *                out_fn MUST NOT call any trace_xxx() function.
 */
void trace_dump_text(void (*out_fn)(const char *line));

/**
 * @brief Dump all buffered events in compact binary format.
 *
 * Binary layout (little-endian):
 *   [4 B]  magic  = 0x54524346  ("TRCF")
 *   [4 B]  version = 1
 *   [4 B]  event_count
 *   [N x 8 B] trace_evt_t records
 *   [4 B]  task_count
 *   [M x (2+16+1) B] task table: {uint16_t id, char name[16], uint8_t prio}
 *
 * @param out_fn  Callback receiving raw bytes.
 */
void trace_dump_binary(void (*out_fn)(const uint8_t *buf, uint32_t len));

/* -----------------------------------------------------------------------
 * Statistics and utility
 * ----------------------------------------------------------------------- */

/** Retrieve accumulated statistics.  May be called while tracing is active. */
void trace_get_stats(trace_stats_t *stats_out);

/** Return the base address of the trace event ring buffer. */
const trace_evt_t *trace_get_buffer_base(void);

/** Return the number of trace_evt_t slots in the ring buffer. */
uint32_t trace_get_buffer_capacity(void);

/** Return the number of valid events currently stored in the ring buffer. */
uint32_t trace_get_event_count(void);

/**
 * @brief Reset ring buffer and statistics without reinitialising HW.
 *
 * Useful for back-to-back captures without reboot.
 * Call only while tracing is disabled.
 */
void trace_reset(void);

/**
 * @brief Get a read-only pointer to a task entry by its trace ID.
 *
 * Returns NULL if id is out of range or slot is unoccupied.
 * Primary use: unit tests and host-side tooling.
 */
const trace_task_entry_t *trace_get_task_entry(uint16_t trace_id);

#ifdef __cplusplus
}
#endif

#endif /* TRACE_FREERTOS_H */
