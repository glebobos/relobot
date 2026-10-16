#include <stdio.h>
#include <time.h>

#include <FreeRTOS.h>
#include <task.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

#include "kamod_led.h"
#include "pico/stdlib.h"

#define APP_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 16)
#define APP_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

static rcl_publisher_t publisher;
static rcl_timer_t timer;
static std_msgs__msg__Float32 message;

static void fatal_blink(unsigned int count)
{
    for (;;) {
        for (unsigned int i = 0; i < count; ++i) {
            kamod_led_set(32, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(150));
            kamod_led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int clock_gettime(clockid_t clock_id, struct timespec *time_point)
{
    (void)clock_id;
    if (time_point == NULL) {
        return -1;
    }

    uint64_t now_us = time_us_64();
    time_point->tv_sec = (time_t)(now_us / 1000000U);
    time_point->tv_nsec = (long)((now_us % 1000000U) * 1000U);
    return 0;
}

static void timer_callback(rcl_timer_t *timer_handle, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer_handle == NULL) {
        return;
    }

    message.data += 1.0f;
    (void)rcl_publish(&publisher, &message, NULL);
}

static void app_task(void *argument)
{
    (void)argument;
    kamod_led_set(32, 16, 0);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
        fatal_blink(1);
    }
    kamod_led_set(0, 0, 32);

    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
        fatal_blink(2);
    }
    if (rcl_init_options_fini(&init_options) != RCL_RET_OK) {
        fatal_blink(3);
    }
    kamod_led_set(32, 32, 32);

    if (rclc_node_init_default(&node, "rmw_zenoh_pico_spike", "", &support) != RCL_RET_OK) {
        fatal_blink(4);
    }
    kamod_led_set(32, 32, 0);

    if (rclc_publisher_init_best_effort(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/rmw_zenoh_pico/value") != RCL_RET_OK) {
        fatal_blink(5);
    }
    if (rclc_timer_init_default(
            &timer, &support, RCL_MS_TO_NS(1000), timer_callback) != RCL_RET_OK) {
        fatal_blink(6);
    }

    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        fatal_blink(7);
    }
    if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
        fatal_blink(8);
    }

    for (;;) {
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}

int main(void)
{
    stdio_init_all();
    kamod_led_init();
    kamod_led_set(32, 0, 32);

    kamod_led_set(32, 32, 32);
    if (xTaskCreate(
            app_task,
            "rmw_zenoh_pico",
            APP_TASK_STACK_WORDS,
            NULL,
            APP_TASK_PRIORITY,
            NULL) != pdPASS) {
        kamod_led_set(32, 0, 0);
        for (;;) {
            tight_loop_contents();
        }
    }
    kamod_led_set(0, 32, 0);
    vTaskStartScheduler();
    kamod_led_set(32, 0, 0);
    for (;;) {
        tight_loop_contents();
    }
}
