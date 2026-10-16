#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>
#include "pico/stdlib.h"
#include "zenoh-pico/runtime/executor.h"

#include "transport_probe.h"

#define TRANSPORT_PROBE_STALL_MS 65000u

static uint32_t executor_progress_ms;
static uint32_t serial_read_started_ms;
static bool executor_seen;
static bool executor_spinning;
static bool serial_read_waiting;
static TaskHandle_t executor_task_handle;
static TaskHandle_t serial_read_task_handle;

_z_executor_spin_result_t __real__z_executor_spin(_z_executor_t *executor);
int __real__z_usb_uart_getc(void);

static void mark_executor_progress(bool spinning)
{
    taskENTER_CRITICAL();
    executor_progress_ms = to_ms_since_boot(get_absolute_time());
    executor_seen = true;
    executor_spinning = spinning;
    executor_task_handle = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();
}

_z_executor_spin_result_t __wrap__z_executor_spin(_z_executor_t *executor)
{
    mark_executor_progress(true);
    _z_executor_spin_result_t result = __real__z_executor_spin(executor);
    mark_executor_progress(false);
    return result;
}

int __wrap__z_usb_uart_getc(void)
{
    taskENTER_CRITICAL();
    serial_read_started_ms = to_ms_since_boot(get_absolute_time());
    serial_read_waiting = true;
    serial_read_task_handle = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();
    int result = __real__z_usb_uart_getc();
    taskENTER_CRITICAL();
    serial_read_waiting = false;
    taskEXIT_CRITICAL();
    return result;
}

transport_probe_status_t transport_probe_status(void)
{
    taskENTER_CRITICAL();
    bool seen = executor_seen;
    uint32_t last_progress_ms = executor_progress_ms;
    bool reading = serial_read_waiting;
    uint32_t read_started_ms = serial_read_started_ms;
    bool same_task = executor_spinning && executor_task_handle == serial_read_task_handle;
    uint32_t current_ms = to_ms_since_boot(get_absolute_time());
    taskEXIT_CRITICAL();
    if (!seen) {
        return TRANSPORT_PROBE_NO_EXECUTOR_PROGRESS;
    }
    if ((uint32_t)(current_ms - last_progress_ms) < TRANSPORT_PROBE_STALL_MS) {
        return TRANSPORT_PROBE_RECENT_PROGRESS;
    }
    if (reading && same_task && (uint32_t)(current_ms - read_started_ms) >= TRANSPORT_PROBE_STALL_MS) {
        return TRANSPORT_PROBE_SERIAL_WAIT;
    }
    return TRANSPORT_PROBE_NO_EXECUTOR_PROGRESS;
}
