#ifndef TRACE_PORT_RCAR_GEN5_H
#define TRACE_PORT_RCAR_GEN5_H

#include <stdint.h>

/*
 * Default trace policy for Renesas R-Car Gen5 CR52:
 * - Generic Timer timestamps
 * - Longer task names
 * - Larger task/ISR tables
 * - Optional fixed-address ring buffer placement
 */

#ifndef TRACE_USE_GENERIC_TIMER
#define TRACE_USE_GENERIC_TIMER 1
#endif

#ifndef TRACE_NAME_LEN
#define TRACE_NAME_LEN 24U
#endif

#ifndef TRACE_MAX_TASKS
#define TRACE_MAX_TASKS 128U
#endif

#ifndef TRACE_MAX_ISRS
#define TRACE_MAX_ISRS 128U
#endif

#ifndef TRACE_MAX_MARKERS
#define TRACE_MAX_MARKERS 128U
#endif

#ifndef configMAX_TASK_NAME_LEN
#define configMAX_TASK_NAME_LEN TRACE_NAME_LEN
#endif

#ifndef INCLUDE_uxTaskPriorityGet
#define INCLUDE_uxTaskPriorityGet 1
#endif

#ifndef INCLUDE_xTaskGetCurrentTaskHandle
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#endif

#ifndef INCLUDE_vTaskDelay
#define INCLUDE_vTaskDelay 1
#endif

#ifndef INCLUDE_vTaskSuspend
#define INCLUDE_vTaskSuspend 1
#endif

#ifdef RCAR_TRACE_BUFFER_BASE
#ifndef TRACE_RING_BUFFER_PTR
#define TRACE_RING_BUFFER_PTR ((trace_evt_t *)(uintptr_t)(RCAR_TRACE_BUFFER_BASE))
#endif
#endif

#ifdef RCAR_TRACE_BUFFER_EVENTS
#ifndef TRACE_RING_BUFFER_CAPACITY
#define TRACE_RING_BUFFER_CAPACITY (RCAR_TRACE_BUFFER_EVENTS)
#endif
#endif

#ifndef TRACE_LOCK
#define TRACE_LOCK() \
    UBaseType_t _trace_saved_mask_ = (UBaseType_t)portSET_INTERRUPT_MASK_FROM_ISR()
#endif

#ifndef TRACE_UNLOCK
#define TRACE_UNLOCK() \
    portCLEAR_INTERRUPT_MASK_FROM_ISR((uint32_t)_trace_saved_mask_)
#endif

#include "trace_freertos.h"

#define traceTASK_SWITCHED_IN()           trace_task_switched_in(pxCurrentTCB)
#define traceTASK_SWITCHED_OUT()          trace_task_switched_out(pxCurrentTCB)
#define traceQUEUE_SEND(pxQueue)          trace_queue_send((void *)(pxQueue))
#define traceQUEUE_RECEIVE(pxQueue)       trace_queue_receive((void *)(pxQueue))
#define traceQUEUE_SEND_FROM_ISR(pxQueue) trace_queue_send((void *)(pxQueue))
#define traceQUEUE_RECEIVE_FROM_ISR(pxQueue) \
    trace_queue_receive((void *)(pxQueue))

#endif
