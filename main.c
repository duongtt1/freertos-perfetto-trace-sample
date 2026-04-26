/*
 * Drop-in trace pipeline sample for:
 *   FreeRTOS/Demo/R-Car_Gen5_CR52/sample_apps/dummy_app
 *
 * Replaces the stock dummy_app with a queue-driven camera pipeline demo and
 * integrates FreeRTOS scheduler/queue/ISR tracing without editing the BSP's
 * global FreeRTOSConfig.h.
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "interrupts.h"
#include "pfc/r_pfc_api.h"
#include "device_tree_x5h.h"
#include "stdio.h"

#include "trace_freertos.h"

enum
{
    CAMERA_COUNT = 4,
    VIN_COUNT = 4,
    ISP_COUNT = 2,
    IMR_COUNT = 10,
    VSPD_COUNT = 4,
    DISPATCH_QUEUE_LENGTH = 64,
    MODULE_QUEUE_LENGTH = 16,
    PIPELINE_TASK_STACK = 1024,
    CAMERA_BASE_PRIORITY = tskIDLE_PRIORITY + 2,
    MODULE_BASE_PRIORITY = tskIDLE_PRIORITY + 3,
    DISPATCHER_PRIORITY = tskIDLE_PRIORITY + 5,
    TRACE_CTRL_PRIORITY = tskIDLE_PRIORITY + 6,
    MARK_CAPTURE_START = 1,
    MARK_CAPTURE_STOP = 2,
    MARK_FRAME_DONE = 3,
    ISR_ID_VIN_BASE = 100,
    ISR_ID_ISP_BASE = 200,
    ISR_ID_IMR_BASE = 300,
    ISR_ID_VSPD_BASE = 400
};

typedef enum
{
    STAGE_TO_VIN = 0,
    STAGE_TO_ISP,
    STAGE_TO_IMR,
    STAGE_TO_VSPD,
    STAGE_COMPLETE
} pipeline_stage_t;

typedef struct
{
    uint32_t frame_id;
    uint8_t cam_id;
    uint8_t vin_id;
    uint8_t isp_id;
    uint8_t imr_id;
    uint8_t vspd_id;
    uint8_t stage;
} frame_msg_t;

typedef struct
{
    uint8_t camera_id;
    TickType_t period_ticks;
} camera_task_ctx_t;

typedef struct
{
    uint8_t module_id;
    uint16_t isr_base;
    TickType_t work_ticks;
    pipeline_stage_t next_stage;
} worker_task_ctx_t;

static QueueHandle_t g_dispatch_queue;
static QueueHandle_t g_vin_queues[VIN_COUNT];
static QueueHandle_t g_isp_queues[ISP_COUNT];
static QueueHandle_t g_imr_queues[IMR_COUNT];
static QueueHandle_t g_vspd_queues[VSPD_COUNT];

static camera_task_ctx_t g_camera_ctx[CAMERA_COUNT];
static worker_task_ctx_t g_vin_ctx[VIN_COUNT];
static worker_task_ctx_t g_isp_ctx[ISP_COUNT];
static worker_task_ctx_t g_imr_ctx[IMR_COUNT];
static worker_task_ctx_t g_vspd_ctx[VSPD_COUNT];

static uint32_t g_frames_completed;
static uint32_t g_dump_done;

static void prvSetupHardware(void);
static void register_trace_metadata(void);
static void init_pipeline_context(void);
static BaseType_t create_pipeline_objects(void);
static BaseType_t create_application_tasks(void);
static BaseType_t create_worker_tasks(const char *prefix,
                                      worker_task_ctx_t *ctx_array,
                                      uint32_t count);
static BaseType_t dispatch_to_stage_queue(const frame_msg_t *msg);
static void uart_write_line(const char *line);

static void DispatcherTask(void *pvParameters);
static void CameraSourceTask(void *pvParameters);
static void ModuleWorkerTask(void *pvParameters);
static void TraceControlTask(void *pvParameters);

int printf_raw(const char *format, ...);

int main(void)
{
    prvSetupHardware();
    trace_init();
    register_trace_metadata();
    init_pipeline_context();

    if ((create_pipeline_objects() != pdPASS) ||
        (create_application_tasks() != pdPASS)) {
        printf("trace_pipeline_app: init failed\n");
        for (;;) {
        }
    }

    trace_enable(true);
    vTaskStartScheduler();

    for (;;) {
    }

    return 0;
}

static void prvSetupHardware(void)
{
    portDISABLE_INTERRUPTS();
    Irq_Setup();
    (void)pfcInitModules(getModuleConfigs());
}

static void register_trace_metadata(void)
{
    char name[TRACE_NAME_LEN];
    uint32_t i;

    trace_register_marker(MARK_CAPTURE_START, "CaptureStart");
    trace_register_marker(MARK_CAPTURE_STOP, "CaptureStop");
    trace_register_marker(MARK_FRAME_DONE, "FrameDone");

    for (i = 0; i < VIN_COUNT; i++) {
        (void)snprintf(name, sizeof(name), "VIN%lu_IRQ", (unsigned long)i);
        trace_register_isr((uint16_t)(ISR_ID_VIN_BASE + i), name);
    }

    for (i = 0; i < ISP_COUNT; i++) {
        (void)snprintf(name, sizeof(name), "ISP%lu_IRQ", (unsigned long)i);
        trace_register_isr((uint16_t)(ISR_ID_ISP_BASE + i), name);
    }

    for (i = 0; i < IMR_COUNT; i++) {
        (void)snprintf(name, sizeof(name), "IMR%lu_IRQ", (unsigned long)i);
        trace_register_isr((uint16_t)(ISR_ID_IMR_BASE + i), name);
    }

    for (i = 0; i < VSPD_COUNT; i++) {
        (void)snprintf(name, sizeof(name), "VSPD%lu_IRQ", (unsigned long)i);
        trace_register_isr((uint16_t)(ISR_ID_VSPD_BASE + i), name);
    }
}

static void init_pipeline_context(void)
{
    static const TickType_t camera_periods[CAMERA_COUNT] = {
        pdMS_TO_TICKS(33),
        pdMS_TO_TICKS(40),
        pdMS_TO_TICKS(50),
        pdMS_TO_TICKS(66)
    };
    uint32_t i;

    for (i = 0; i < CAMERA_COUNT; i++) {
        g_camera_ctx[i].camera_id = (uint8_t)i;
        g_camera_ctx[i].period_ticks = camera_periods[i];
    }

    for (i = 0; i < VIN_COUNT; i++) {
        g_vin_ctx[i].module_id = (uint8_t)i;
        g_vin_ctx[i].isr_base = ISR_ID_VIN_BASE;
        g_vin_ctx[i].work_ticks = pdMS_TO_TICKS(2 + (i % 2));
        g_vin_ctx[i].next_stage = STAGE_TO_ISP;
    }

    for (i = 0; i < ISP_COUNT; i++) {
        g_isp_ctx[i].module_id = (uint8_t)i;
        g_isp_ctx[i].isr_base = ISR_ID_ISP_BASE;
        g_isp_ctx[i].work_ticks = pdMS_TO_TICKS(3 + i);
        g_isp_ctx[i].next_stage = STAGE_TO_IMR;
    }

    for (i = 0; i < IMR_COUNT; i++) {
        g_imr_ctx[i].module_id = (uint8_t)i;
        g_imr_ctx[i].isr_base = ISR_ID_IMR_BASE;
        g_imr_ctx[i].work_ticks = pdMS_TO_TICKS(1 + (i % 3));
        g_imr_ctx[i].next_stage = STAGE_TO_VSPD;
    }

    for (i = 0; i < VSPD_COUNT; i++) {
        g_vspd_ctx[i].module_id = (uint8_t)i;
        g_vspd_ctx[i].isr_base = ISR_ID_VSPD_BASE;
        g_vspd_ctx[i].work_ticks = pdMS_TO_TICKS(2 + (i % 2));
        g_vspd_ctx[i].next_stage = STAGE_COMPLETE;
    }
}

static BaseType_t create_pipeline_objects(void)
{
    uint32_t i;

    g_dispatch_queue = xQueueCreate(DISPATCH_QUEUE_LENGTH, sizeof(frame_msg_t));
    if (g_dispatch_queue == NULL) {
        return pdFAIL;
    }

    for (i = 0; i < VIN_COUNT; i++) {
        g_vin_queues[i] = xQueueCreate(MODULE_QUEUE_LENGTH, sizeof(frame_msg_t));
        if (g_vin_queues[i] == NULL) {
            return pdFAIL;
        }
    }

    for (i = 0; i < ISP_COUNT; i++) {
        g_isp_queues[i] = xQueueCreate(MODULE_QUEUE_LENGTH, sizeof(frame_msg_t));
        if (g_isp_queues[i] == NULL) {
            return pdFAIL;
        }
    }

    for (i = 0; i < IMR_COUNT; i++) {
        g_imr_queues[i] = xQueueCreate(MODULE_QUEUE_LENGTH, sizeof(frame_msg_t));
        if (g_imr_queues[i] == NULL) {
            return pdFAIL;
        }
    }

    for (i = 0; i < VSPD_COUNT; i++) {
        g_vspd_queues[i] = xQueueCreate(MODULE_QUEUE_LENGTH, sizeof(frame_msg_t));
        if (g_vspd_queues[i] == NULL) {
            return pdFAIL;
        }
    }

    return pdPASS;
}

static BaseType_t create_worker_tasks(const char *prefix,
                                      worker_task_ctx_t *ctx_array,
                                      uint32_t count)
{
    uint32_t i;

    for (i = 0; i < count; i++) {
        char name[configMAX_TASK_NAME_LEN];

        (void)snprintf(name, sizeof(name), "%s%lu", prefix, (unsigned long)i);
        if (xTaskCreate(ModuleWorkerTask,
                        name,
                        PIPELINE_TASK_STACK,
                        &ctx_array[i],
                        MODULE_BASE_PRIORITY,
                        NULL) != pdPASS) {
            return pdFAIL;
        }
    }

    return pdPASS;
}

static BaseType_t create_application_tasks(void)
{
    uint32_t i;

    if (xTaskCreate(DispatcherTask,
                    "Dispatch",
                    PIPELINE_TASK_STACK,
                    NULL,
                    DISPATCHER_PRIORITY,
                    NULL) != pdPASS) {
        return pdFAIL;
    }

    for (i = 0; i < CAMERA_COUNT; i++) {
        char name[configMAX_TASK_NAME_LEN];

        (void)snprintf(name, sizeof(name), "Cam%lu", (unsigned long)i);
        if (xTaskCreate(CameraSourceTask,
                        name,
                        PIPELINE_TASK_STACK,
                        &g_camera_ctx[i],
                        CAMERA_BASE_PRIORITY,
                        NULL) != pdPASS) {
            return pdFAIL;
        }
    }

    if (create_worker_tasks("VIN", g_vin_ctx, VIN_COUNT) != pdPASS) {
        return pdFAIL;
    }
    if (create_worker_tasks("ISP", g_isp_ctx, ISP_COUNT) != pdPASS) {
        return pdFAIL;
    }
    if (create_worker_tasks("IMR", g_imr_ctx, IMR_COUNT) != pdPASS) {
        return pdFAIL;
    }
    if (create_worker_tasks("VSPD", g_vspd_ctx, VSPD_COUNT) != pdPASS) {
        return pdFAIL;
    }

    if (xTaskCreate(TraceControlTask,
                    "TraceCtrl",
                    PIPELINE_TASK_STACK,
                    NULL,
                    TRACE_CTRL_PRIORITY,
                    NULL) != pdPASS) {
        return pdFAIL;
    }

    return pdPASS;
}

static BaseType_t dispatch_to_stage_queue(const frame_msg_t *msg)
{
    if (msg == NULL) {
        return pdFAIL;
    }

    switch ((pipeline_stage_t)msg->stage) {
        case STAGE_TO_VIN:
            return xQueueSend(g_vin_queues[msg->vin_id], msg, portMAX_DELAY);

        case STAGE_TO_ISP:
            return xQueueSend(g_isp_queues[msg->isp_id], msg, portMAX_DELAY);

        case STAGE_TO_IMR:
            return xQueueSend(g_imr_queues[msg->imr_id], msg, portMAX_DELAY);

        case STAGE_TO_VSPD:
            return xQueueSend(g_vspd_queues[msg->vspd_id], msg, portMAX_DELAY);

        case STAGE_COMPLETE:
            g_frames_completed++;
            trace_marker(MARK_FRAME_DONE);
            return pdPASS;

        default:
            return pdFAIL;
    }
}

static void DispatcherTask(void *pvParameters)
{
    frame_msg_t msg;

    (void)pvParameters;

    for (;;) {
        if (xQueueReceive(g_dispatch_queue, &msg, portMAX_DELAY) == pdPASS) {
            (void)dispatch_to_stage_queue(&msg);
        }
    }
}

static void CameraSourceTask(void *pvParameters)
{
    camera_task_ctx_t *ctx = (camera_task_ctx_t *)pvParameters;
    uint32_t frame_seq = 0U;

    for (;;) {
        frame_msg_t msg;

        frame_seq++;
        msg.frame_id = ((uint32_t)ctx->camera_id << 24U) | frame_seq;
        msg.cam_id = ctx->camera_id;
        msg.vin_id = ctx->camera_id;
        msg.isp_id = (uint8_t)(ctx->camera_id % ISP_COUNT);
        msg.imr_id = (uint8_t)((frame_seq + ctx->camera_id) % IMR_COUNT);
        msg.vspd_id = (uint8_t)(ctx->camera_id % VSPD_COUNT);
        msg.stage = STAGE_TO_VIN;

        trace_isr_enter((uint16_t)(ISR_ID_VIN_BASE + ctx->camera_id));
        (void)xQueueSend(g_dispatch_queue, &msg, portMAX_DELAY);
        trace_isr_exit((uint16_t)(ISR_ID_VIN_BASE + ctx->camera_id));

        vTaskDelay(ctx->period_ticks);
    }
}

static void ModuleWorkerTask(void *pvParameters)
{
    worker_task_ctx_t *ctx = (worker_task_ctx_t *)pvParameters;
    QueueHandle_t input_queue = NULL;
    frame_msg_t msg;

    switch (ctx->next_stage) {
        case STAGE_TO_ISP:
            input_queue = g_vin_queues[ctx->module_id];
            break;

        case STAGE_TO_IMR:
            input_queue = g_isp_queues[ctx->module_id];
            break;

        case STAGE_TO_VSPD:
            input_queue = g_imr_queues[ctx->module_id];
            break;

        case STAGE_COMPLETE:
            input_queue = g_vspd_queues[ctx->module_id];
            break;

        default:
            break;
    }

    for (;;) {
        if ((input_queue != NULL) &&
            (xQueueReceive(input_queue, &msg, portMAX_DELAY) == pdPASS)) {
            vTaskDelay(ctx->work_ticks);
            msg.stage = (uint8_t)ctx->next_stage;

            trace_isr_enter((uint16_t)(ctx->isr_base + ctx->module_id));
            (void)xQueueSend(g_dispatch_queue, &msg, portMAX_DELAY);
            trace_isr_exit((uint16_t)(ctx->isr_base + ctx->module_id));
        }
    }
}

static void uart_write_line(const char *line)
{
    if (line != NULL) {
        printf("%s\n", line);
    }
}

static void TraceControlTask(void *pvParameters)
{
    trace_stats_t stats;

    (void)pvParameters;

    trace_marker(MARK_CAPTURE_START);
    vTaskDelay(pdMS_TO_TICKS(RCAR_TRACE_CAPTURE_MS));
    trace_marker(MARK_CAPTURE_STOP);
    trace_enable(false);

    if (g_dump_done == 0U) {
        g_dump_done = 1U;
        printf("#UART_TRACE_BEGIN\n");
        trace_dump_text(uart_write_line);
        printf("#UART_TRACE_END\n");
        trace_get_stats(&stats);
        printf("#SUMMARY frames_completed=%lu recorded=%lu dropped=%lu peak=%lu\n",
               (unsigned long)g_frames_completed,
               (unsigned long)stats.events_recorded,
               (unsigned long)stats.events_dropped,
               (unsigned long)stats.buffer_peak);
    }

    vTaskSuspend(NULL);
}

void vMainAssertCalled(const char *pcFileName, uint32_t ulLineNumber)
{
    printf_raw("ASSERT!  Line %lu of file %s\n",
               (unsigned long)ulLineNumber,
               pcFileName);
    taskENTER_CRITICAL();
    for (;;) {
    }
}

void vDeleteCallingTask(void)
{
    vTaskDelete(NULL);
}
