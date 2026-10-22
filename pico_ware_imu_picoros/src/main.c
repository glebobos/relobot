#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <picoros.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "tusb.h"

#include "imu_sensor.h"
#include "imu_codec.h"
#include "float32_codec.h"
#include "relobot/board.h"
#include "relobot/picoros_runtime.h"

#define TELEMETRY_PERIOD_MS         33      /* 30 Hz publish rate */
#define WATCHDOG_TIMEOUT_MS         2000    /* 2 s hardware watchdog */
#define IMU_STALE_THRESHOLD_MS      500     /* ~15 missed 30 Hz samples before a field group is marked unavailable */

/* Mirror the diagonal defaults set once in imu_msg_init_defaults() (imu_codec.c) */
#define ORIENTATION_COVARIANCE_DIAG          0.001
#define ANGULAR_VELOCITY_COVARIANCE_DIAG     0.0001
#define LINEAR_ACCELERATION_COVARIANCE_DIAG  0.01

#define APP_TASK_STACK_WORDS        (configMINIMAL_STACK_SIZE * 6)
#define SENSOR_TASK_STACK_WORDS     (configMINIMAL_STACK_SIZE * 4)
#define APP_TASK_PRIORITY           (tskIDLE_PRIORITY + 2)
#define SENSOR_TASK_PRIORITY        (APP_TASK_PRIORITY + 1)

static TaskHandle_t app_task_handle;
static TaskHandle_t sensor_task_handle;

static bool entities_ready = false;
static picoros_node_t node = {.name = "imu_node"};
static volatile unsigned int diagnostic_stage = 1;

void picoros_diagnostic_stage(unsigned int stage)
{
    taskENTER_CRITICAL();
    diagnostic_stage = stage;
    taskEXIT_CRITICAL();
}

static picoros_publisher_t pub_imu = {
    .topic = {
        .name = "imu",
        .type = "sensor_msgs::msg::dds_::Imu",
    },
};

#if RELOBOT_IMU_BENCH
static picoros_publisher_t bench_publishers[] = {
    {.topic = {.name = "imu/bench/heap_free_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "imu/bench/heap_min_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "imu/bench/app_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "imu/bench/sensor_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "imu/bench/publish_errors", .type = "std_msgs::msg::dds_::Float32"}},
};
#endif

static char frame_id_buf[] = "imu_link";
static sensor_msgs__msg__Imu imu_msg;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

/* Tracks per-field-group freshness. Returns true once `threshold_ms` has
 * elapsed since the last valid reading of this field group, so a dead
 * sensor cannot make the node republish stale data as if it were live. */
static bool update_freshness(uint32_t *last_update_ms, bool is_valid, uint32_t now, uint32_t threshold_ms)
{
    if (is_valid) {
        *last_update_ms = now;
    }
    return (uint32_t)(now - *last_update_ms) >= threshold_ms;
}

void relobot_fault(void)
{
    (void)save_and_disable_interrupts();
    watchdog_reboot(0, 0, 0);
    for (;;) { tight_loop_contents(); }
}

void vApplicationMallocFailedHook(void)
{
    relobot_fault();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    relobot_fault();
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
    (void)itf;
    if (p_line_coding && p_line_coding->bit_rate == 1200) {
        reset_usb_boot(0, 0);
    }
}

#if RELOBOT_IMU_BENCH
static bool publish_float(picoros_publisher_t *publisher, float value)
{
    uint8_t payload[FLOAT32_CDR_SIZE];
    return float32_codec_encode(value, payload) &&
           picoros_publish(publisher, payload, sizeof(payload)) == PICOROS_OK;
}
#endif

static void startup_failed(unsigned int stage)
{
    taskENTER_CRITICAL();
    entities_ready = false;
    taskEXIT_CRITICAL();
    picoros_diagnostic_stage(stage);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void app_task(void *argument)
{
    (void)argument;

    if (!imu_codec_init() || !float32_codec_init()) {
        startup_failed(0);
    }

    pub_imu.topic.rihs_hash = imu_codec_hash();

    for (;;) {
        picoros_diagnostic_stage(1);
        if (relobot_picoros_init() != PICOROS_OK) {
            picoros_diagnostic_stage(3);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        picoros_diagnostic_stage(7);
        if (picoros_node_init(&node) != PICOROS_OK) {
            picoros_interface_close();
            picoros_diagnostic_stage(4);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        picoros_diagnostic_stage(8);

        z_publisher_options_default(&pub_imu.opts);
        if (picoros_publisher_declare(&node, &pub_imu) != PICOROS_OK) {
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

#if RELOBOT_IMU_BENCH
        bool bench_ok = true;
        size_t declared_bench = 0;
        for (size_t index = 0; index < sizeof(bench_publishers) / sizeof(bench_publishers[0]); ++index) {
            bench_publishers[index].topic.rihs_hash = float32_codec_hash();
            z_publisher_options_default(&bench_publishers[index].opts);
            if (picoros_publisher_declare(&node, &bench_publishers[index]) != PICOROS_OK) {
                bench_ok = false;
                break;
            }
            declared_bench = index + 1;
        }
        if (!bench_ok) {
            for (size_t index = 0; index < declared_bench; ++index) {
                (void)picoros_publisher_drop(&bench_publishers[index]);
            }
            (void)picoros_publisher_drop(&pub_imu);
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
#endif
        break;
    }

    taskENTER_CRITICAL();
    entities_ready = true;
    diagnostic_stage = 10;
    taskEXIT_CRITICAL();

    /* App task remains running to maintain FreeRTOS context */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void sensor_task(void *argument)
{
    (void)argument;
    TickType_t previous_wake = xTaskGetTickCount();

    uint32_t last_orientation_ms = now_ms();
    uint32_t last_angular_velocity_ms = last_orientation_ms;
    uint32_t last_linear_acceleration_ms = last_orientation_ms;

#if RELOBOT_IMU_BENCH
    uint32_t publish_errors = 0;
    uint32_t last_diagnostics_ms = now_ms();
#endif

    for (;;) {
        taskENTER_CRITICAL();
        bool ready = entities_ready;
        unsigned int stage = diagnostic_stage;
        taskEXIT_CRITICAL();

        imu_sensor_task();

        uint32_t now = now_ms();
        imu_sensor_data_t data;
        bool orientation_valid = false;
        bool angular_velocity_valid = false;
        bool linear_acceleration_valid = false;
        if (imu_sensor_read_data(&data)) {
            if (data.orientation_valid) {
                imu_msg.orientation.w = data.orientation[0];
                imu_msg.orientation.x = data.orientation[1];
                imu_msg.orientation.y = data.orientation[2];
                imu_msg.orientation.z = data.orientation[3];
                orientation_valid = true;
            }
            if (data.angular_velocity_valid) {
                imu_msg.angular_velocity.x = data.angular_velocity[0];
                imu_msg.angular_velocity.y = data.angular_velocity[1];
                imu_msg.angular_velocity.z = data.angular_velocity[2];
                angular_velocity_valid = true;
            }
            if (data.linear_acceleration_valid) {
                imu_msg.linear_acceleration.x = data.linear_acceleration[0];
                imu_msg.linear_acceleration.y = data.linear_acceleration[1];
                imu_msg.linear_acceleration.z = data.linear_acceleration[2];
                linear_acceleration_valid = true;
            }
        }

        /* A field group that has gone stale is marked unavailable via the
         * sensor_msgs/Imu covariance[0]=-1 convention instead of silently
         * republishing the last-known reading forever. */
        bool orientation_stale = update_freshness(&last_orientation_ms,
            orientation_valid, now, IMU_STALE_THRESHOLD_MS);
        bool angular_velocity_stale = update_freshness(&last_angular_velocity_ms,
            angular_velocity_valid, now, IMU_STALE_THRESHOLD_MS);
        bool linear_acceleration_stale = update_freshness(&last_linear_acceleration_ms,
            linear_acceleration_valid, now, IMU_STALE_THRESHOLD_MS);
        bool any_stale = orientation_stale || angular_velocity_stale || linear_acceleration_stale;

        imu_msg.orientation_covariance[0] =
            orientation_stale ? -1.0 : ORIENTATION_COVARIANCE_DIAG;
        imu_msg.angular_velocity_covariance[0] =
            angular_velocity_stale ? -1.0 : ANGULAR_VELOCITY_COVARIANCE_DIAG;
        imu_msg.linear_acceleration_covariance[0] =
            linear_acceleration_stale ? -1.0 : LINEAR_ACCELERATION_COVARIANCE_DIAG;

        if (ready) {
            uint64_t now_us = to_us_since_boot(get_absolute_time());
            imu_msg.header.stamp.sec = (int32_t)(now_us / 1000000ULL);
            imu_msg.header.stamp.nanosec = (uint32_t)((now_us % 1000000ULL) * 1000ULL);

            uint8_t payload[IMU_CDR_MAX_SIZE];
            size_t actual_length = 0;
            if (imu_codec_encode(&imu_msg, payload, sizeof(payload), &actual_length)) {
                if (picoros_publish(&pub_imu, payload, actual_length) != PICOROS_OK) {
#if RELOBOT_IMU_BENCH
                    publish_errors++;
#endif
                }
            } else {
#if RELOBOT_IMU_BENCH
                publish_errors++;
#endif
            }

#if RELOBOT_IMU_BENCH
            if ((uint32_t)(now - last_diagnostics_ms) >= 1000) {
                float values[] = {
                    (float)xPortGetFreeHeapSize(),
                    (float)xPortGetMinimumEverFreeHeapSize(),
                    (float)uxTaskGetStackHighWaterMark(app_task_handle),
                    (float)uxTaskGetStackHighWaterMark(sensor_task_handle),
                    (float)publish_errors,
                };
                _Static_assert(sizeof(values) / sizeof(values[0]) ==
                               sizeof(bench_publishers) / sizeof(bench_publishers[0]),
                               "bench values must match publishers");
                for (size_t index = 0; index < sizeof(values) / sizeof(values[0]); ++index) {
                    if (!publish_float(&bench_publishers[index], values[index])) {
                        publish_errors++;
                        break;
                    }
                }
                last_diagnostics_ms = now;
            }
#endif
        }

        /* LED status visualization */
        uint8_t r = 0, g = 0, b = 0;
        if ((now % 1000) < 100) {
            /* Heartbeat: brief 100 ms off phase every second */
            r = 0; g = 0; b = 0;
        } else if (stage == 0) {
            b = 32; /* Solid Blue: codec_init failed */
        } else if (stage >= 1 && stage <= 5) {
            r = 32; /* Red: waiting/attempting Zenoh session init */
        } else if (ready && stage >= 6 && stage <= 10) {
            if (any_stale) {
                r = 24; g = 24; /* Amber: publishing, but sensor data is stale */
            } else {
                g = 32; /* Solid Green: active publishing */
            }
        } else if (stage >= 6 && stage <= 9) {
            r = 24; g = 20; /* Amber/Yellow: Zenoh session open, declaring ROS entities */
        } else {
            r = 32; b = 16; /* Magenta: unexpected state */
        }

        relobot_board_led_set(r, g, b);
        watchdog_update();
        vTaskDelayUntil(&previous_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

int main(void)
{
    imu_sensor_hardware_init();
    watchdog_enable(WATCHDOG_TIMEOUT_MS, false);
    relobot_board_init();
    imu_msg_init_defaults(&imu_msg, frame_id_buf, sizeof(frame_id_buf));

    if (xTaskCreate(sensor_task, "imu_sensor", SENSOR_TASK_STACK_WORDS,
                    NULL, SENSOR_TASK_PRIORITY, &sensor_task_handle) != pdPASS ||
        xTaskCreate(app_task, "picoros_app", APP_TASK_STACK_WORDS,
                    NULL, APP_TASK_PRIORITY, &app_task_handle) != pdPASS) {
        for (;;) { tight_loop_contents(); }
    }

    vTaskStartScheduler();

    for (;;) { tight_loop_contents(); }
}
