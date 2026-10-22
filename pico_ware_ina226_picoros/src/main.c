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

#include "ina226.h"
#include "float32_codec.h"
#include "bool_codec.h"
#include "relobot/board.h"
#include "relobot/picoros_runtime.h"

#define TELEMETRY_PERIOD_MS         100     /* 10 Hz publish rate */
#define WATCHDOG_TIMEOUT_MS         2000    /* 2 s hardware watchdog */

#define APP_TASK_STACK_WORDS        (configMINIMAL_STACK_SIZE * 6)
#define SENSOR_TASK_STACK_WORDS     (configMINIMAL_STACK_SIZE)
#define APP_TASK_PRIORITY           (tskIDLE_PRIORITY + 2)
#define SENSOR_TASK_PRIORITY        (APP_TASK_PRIORITY + 1)

static TaskHandle_t app_task_handle;
static TaskHandle_t sensor_task_handle;

static bool entities_ready = false;
static picoros_node_t node = {.name = "ina226_node"};
static volatile unsigned int diagnostic_stage = 1;

void picoros_diagnostic_stage(unsigned int stage)
{
    taskENTER_CRITICAL();
    diagnostic_stage = stage;
    taskEXIT_CRITICAL();
}

static picoros_publisher_t pub_battery_voltage = {
    .topic = {
        .name = "battery_voltage",
        .type = "std_msgs::msg::dds_::Float32",
    },
};

static picoros_publisher_t pub_charger_voltage = {
    .topic = {
        .name = "charger_voltage",
        .type = "std_msgs::msg::dds_::Float32",
    },
};

static picoros_publisher_t pub_on_dock = {
    .topic = {
        .name = "on_dock",
        .type = "std_msgs::msg::dds_::Bool",
    },
};

#if RELOBOT_INA226_BENCH
static picoros_publisher_t bench_publishers[] = {
    {.topic = {.name = "ina226/bench/heap_free_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "ina226/bench/heap_min_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "ina226/bench/app_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "ina226/bench/sensor_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "ina226/bench/publish_errors", .type = "std_msgs::msg::dds_::Float32"}},
};
#endif

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
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

static bool publish_float(picoros_publisher_t *publisher, float value)
{
    uint8_t payload[FLOAT32_CDR_SIZE];
    return float32_codec_encode(value, payload) &&
           picoros_publish(publisher, payload, sizeof(payload)) == PICOROS_OK;
}

static bool publish_bool(picoros_publisher_t *publisher, bool value)
{
    uint8_t payload[BOOL_CDR_SIZE];
    size_t actual_len = 0;
    return bool_codec_encode(value, payload, &actual_len) &&
           picoros_publish(publisher, payload, actual_len) == PICOROS_OK;
}

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

    if (!float32_codec_init() || !bool_codec_init()) {
        startup_failed(0);
    }

    pub_battery_voltage.topic.rihs_hash = float32_codec_hash();
    pub_charger_voltage.topic.rihs_hash = float32_codec_hash();
    pub_on_dock.topic.rihs_hash = bool_codec_hash();

    picoros_diagnostic_stage(1);
    if (relobot_picoros_init() != PICOROS_OK) {
        startup_failed(3);
    }
    picoros_diagnostic_stage(7);
    if (picoros_node_init(&node) != PICOROS_OK) {
        startup_failed(4);
    }
    picoros_diagnostic_stage(8);

    z_publisher_options_default(&pub_battery_voltage.opts);
    if (picoros_publisher_declare(&node, &pub_battery_voltage) != PICOROS_OK) {
        startup_failed(5);
    }

    z_publisher_options_default(&pub_charger_voltage.opts);
    if (picoros_publisher_declare(&node, &pub_charger_voltage) != PICOROS_OK) {
        startup_failed(5);
    }

    z_publisher_options_default(&pub_on_dock.opts);
    if (picoros_publisher_declare(&node, &pub_on_dock) != PICOROS_OK) {
        startup_failed(5);
    }

#if RELOBOT_INA226_BENCH
    for (size_t index = 0; index < sizeof(bench_publishers) / sizeof(bench_publishers[0]); ++index) {
        bench_publishers[index].topic.rihs_hash = float32_codec_hash();
        z_publisher_options_default(&bench_publishers[index].opts);
        if (picoros_publisher_declare(&node, &bench_publishers[index]) != PICOROS_OK) {
            startup_failed(5);
        }
    }
#endif

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

#if RELOBOT_INA226_BENCH
    uint32_t publish_errors = 0;
    uint32_t last_diagnostics_ms = now_ms();
#endif

    for (;;) {
        taskENTER_CRITICAL();
        bool ready = entities_ready;
        unsigned int stage = diagnostic_stage;
        taskEXIT_CRITICAL();

        float v_battery = ina226_read_bus_voltage(INA226_ADDR_BATTERY);
        float v_charger = ina226_read_bus_voltage(INA226_ADDR_CHARGER);
        bool on_dock = ina226_is_on_dock(v_charger);

        if (ready) {
            /* Skip invalid readings so subscribers never see I2C error values as data */
            if (!isnan(v_battery)) {
                if (!publish_float(&pub_battery_voltage, v_battery)) {
#if RELOBOT_INA226_BENCH
                    publish_errors++;
#endif
                }
            }
            if (!isnan(v_charger)) {
                if (!publish_float(&pub_charger_voltage, v_charger)) {
#if RELOBOT_INA226_BENCH
                    publish_errors++;
#endif
                }
            }
            /* Always publish dock status — false on I2C error is the safe default */
            if (!publish_bool(&pub_on_dock, on_dock)) {
#if RELOBOT_INA226_BENCH
                publish_errors++;
#endif
            }

#if RELOBOT_INA226_BENCH
            uint32_t now = now_ms();
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
        uint32_t now = now_ms();
        uint8_t r = 0, g = 0, b = 0;
        if ((now % 1000) < 100) {
            /* Heartbeat: brief 100 ms off phase every second */
            r = 0; g = 0; b = 0;
        } else if (stage == 0) {
            b = 32; /* Solid Blue: codec_init failed */
        } else if (stage >= 1 && stage <= 5) {
            r = 32; /* Red: waiting/attempting Zenoh session init */
        } else if (stage >= 6 && stage <= 9) {
            r = 24; g = 20; /* Amber/Yellow: Zenoh session open, declaring ROS entities */
        } else if (stage == 10) {
            if (!ready) {
                r = 32;
            } else {
                g = 32; /* Solid Green: active publishing */
            }
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
    ina226_hardware_init();
    watchdog_enable(WATCHDOG_TIMEOUT_MS, false);
    relobot_board_init();

    if (xTaskCreate(sensor_task, "ina226_sensor", SENSOR_TASK_STACK_WORDS,
                    NULL, SENSOR_TASK_PRIORITY, &sensor_task_handle) != pdPASS ||
        xTaskCreate(app_task, "picoros_app", APP_TASK_STACK_WORDS,
                    NULL, APP_TASK_PRIORITY, &app_task_handle) != pdPASS) {
        for (;;) { tight_loop_contents(); }
    }

    vTaskStartScheduler();

    for (;;) { tight_loop_contents(); }
}
