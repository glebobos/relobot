#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <picoros.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "tusb.h"

#include "float32_codec.h"
#include "command_state.h"
#include "pid_controller.h"
#include "relobot/board.h"
#include "relobot/picoros_runtime.h"

#define PWM_WRAP                    12499
#define CONTROL_PERIOD_MS           20
#define RPM_SAMPLE_PERIOD_MS        600
#define TELEMETRY_PERIOD_MS         100
#define WATCHDOG_TIMEOUT_MS         2000
#define MAX_CONTROL_GAP_MS          100
#define ENCODER_TIMEOUT_MS          2000
#define BRAKE_MAX_DURATION_MS       600

#define BRAKE_RPM                   300.0f
#define BRAKE_THRESHOLD_RPM         100.0f
#define COUNTS_PER_REVOLUTION       1.0f

#define FEED_FORWARD_K              0.001381f
#define FEED_FORWARD_C              0.048623f

#define PI_KP                       0.00008f
#define PI_KI                       0.00002f
#define PI_INTEGRAL_LIMIT           1000.0f
#define PI_OUTPUT_LIMIT             0.20f

#define RAMP_RATE_RPM_PER_SEC       2500.0f

#define PIN_FWD_PWM                 26
#define PIN_FWD_EN                  27
#define PIN_REV_PWM                 29
#define PIN_REV_EN                  6
#define PIN_ENCODER                 7

#define DIRECTION_DEADTIME_MS       2
#define DIRECTION_CHANGE_RPM        50.0f

#define APP_TASK_STACK_WORDS        (configMINIMAL_STACK_SIZE * 6)
#define CONTROL_TASK_STACK_WORDS    (configMINIMAL_STACK_SIZE)
#define APP_TASK_PRIORITY           (tskIDLE_PRIORITY + 2)
#define CONTROL_TASK_PRIORITY       (APP_TASK_PRIORITY + 1)

static TaskHandle_t app_task_handle;
static TaskHandle_t control_task_handle;

static volatile uint32_t encoder_ticks = 0;

static knives_command_state_t command_state;
static float control_rpm = 0.0f;
static float measured_rpm = 0.0f;
static pid_controller_t rpm_pid;

static uint32_t last_control_ms = 0;
static uint32_t last_rpm_sample_ms = 0;
static uint32_t last_encoder_ticks = 0;
static uint32_t last_activity_encoder_ticks = 0;
static uint32_t last_encoder_activity_ms = 0;
static uint32_t braking_started_ms = 0;

static bool is_braking = false;
static bool brake_forward = true;
static bool last_dir_forward = true;
static bool new_measurement_available = false;
static float last_pwm_adjust = 0.0f;
static bool entities_ready = false;

static picoros_node_t node = {.name = "knives_node"};
static volatile unsigned int diagnostic_stage = 1;

void picoros_diagnostic_stage(unsigned int stage)
{
    taskENTER_CRITICAL();
    diagnostic_stage = stage;
    taskEXIT_CRITICAL();
}

static picoros_publisher_t pub_current_rpm = {
    .topic = {
        .name = "knives/current_rpm",
        .type = "std_msgs::msg::dds_::Float32",
    },
};

typedef enum {
    KNIVES_FAULT_NONE            = 0,
    KNIVES_FAULT_CMD_TIMEOUT     = 1,
    KNIVES_FAULT_GAP_TIMEOUT     = 2,
    KNIVES_FAULT_ENCODER_TIMEOUT = 3,
    KNIVES_FAULT_BRAKE_TIMEOUT   = 4,
    KNIVES_FAULT_UNARMED_REJECT  = 5,
    KNIVES_FAULT_DECODE_ERR      = 6,
    KNIVES_FAULT_NON_FINITE      = 7,
} knives_fault_code_t;

static picoros_publisher_t debug_publishers[] = {
    {.topic = {.name = "knives/debug/target_rpm", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/control_rpm", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/out_pwm", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/encoder_ticks", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/armed", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/fault_code", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/cmd_count", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/last_raw_cmd", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/debug/last_cmd_age_ms", .type = "std_msgs::msg::dds_::Float32"}},
};

static volatile uint32_t debug_cmd_count = 0;
static volatile float debug_last_raw_cmd = 0.0f;
static volatile float debug_out_pwm = 0.0f;
static volatile float debug_fault_code = 0.0f;

#if RELOBOT_KNIVES_BENCH
static picoros_publisher_t bench_publishers[] = {
    {.topic = {.name = "knives/bench/target_rpm", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/bench/heap_free_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/bench/heap_min_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/bench/app_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/bench/control_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "knives/bench/publish_errors", .type = "std_msgs::msg::dds_::Float32"}},
};
#endif

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static float rpm_to_base_pwm(float rpm)
{
    float omega = fabsf(rpm) * (2.0f * (float)M_PI / 60.0f);
    return pid_clampf(FEED_FORWARD_K * omega + FEED_FORWARD_C, 0.0f, 1.0f);
}

static void set_motor_pwm(float pwm, bool forward)
{
    debug_out_pwm = (pwm > 0.0f) ? pwm : 0.0f;
#if RELOBOT_KNIVES_BENCH
    (void)pwm;
    (void)forward;
#else
    uint16_t level = (uint16_t)(pid_clampf(pwm, 0.0f, 1.0f) * (float)PWM_WRAP);

    if (level == 0) {
        gpio_put(PIN_FWD_EN, 0);
        gpio_put(PIN_REV_EN, 0);
        pwm_set_gpio_level(PIN_FWD_PWM, 0);
        pwm_set_gpio_level(PIN_REV_PWM, 0);
        last_dir_forward = forward;
        return;
    }

    if (forward != last_dir_forward) {
        gpio_put(PIN_FWD_EN, 0);
        gpio_put(PIN_REV_EN, 0);
        pwm_set_gpio_level(PIN_FWD_PWM, 0);
        pwm_set_gpio_level(PIN_REV_PWM, 0);
        sleep_ms(DIRECTION_DEADTIME_MS);
        last_dir_forward = forward;
    }

    gpio_put(PIN_FWD_EN, 1);
    gpio_put(PIN_REV_EN, 1);

    if (forward) {
        pwm_set_gpio_level(PIN_FWD_PWM, level);
        pwm_set_gpio_level(PIN_REV_PWM, 0);
    } else {
        pwm_set_gpio_level(PIN_FWD_PWM, 0);
        pwm_set_gpio_level(PIN_REV_PWM, level);
    }
#endif
}

static void stop_motor_control(void)
{
    debug_out_pwm = 0.0f;
    control_rpm = 0.0f;
    is_braking = false;
    last_pwm_adjust = 0.0f;
    new_measurement_available = false;
    pid_controller_reset(&rpm_pid);
    set_motor_pwm(0.0f, true);
}

static void emergency_stop(void)
{
    knives_command_disarm(&command_state);
    stop_motor_control();
}

void relobot_fault(void)
{
    (void)save_and_disable_interrupts();
    set_motor_pwm(0.0f, true);
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

static void control_fault(void)
{
    taskENTER_CRITICAL();
    knives_command_disarm(&command_state);
    taskEXIT_CRITICAL();
    stop_motor_control();
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
    (void)itf;
    if (p_line_coding && p_line_coding->bit_rate == 1200) {
        emergency_stop();
        reset_usb_boot(0, 0);
    }
}

#if !RELOBOT_KNIVES_BENCH
static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == PIN_ENCODER) {
        encoder_ticks++;
    }
}
#endif

static void command_received(uint8_t *payload, size_t length)
{
    debug_cmd_count++;
    float value;
    if (!float32_codec_decode(payload, length, &value)) {
        debug_fault_code = (float)KNIVES_FAULT_DECODE_ERR;
        return;
    }
    debug_last_raw_cmd = value;
    if (!isfinite(value)) {
        debug_fault_code = (float)KNIVES_FAULT_NON_FINITE;
        return;
    }
    taskENTER_CRITICAL();
    bool accepted = knives_command_accept(&command_state, value, now_ms());
    if (!accepted && !command_state.armed && fabsf(value) >= KNIVES_ZERO_EPSILON_RPM) {
        debug_fault_code = (float)KNIVES_FAULT_UNARMED_REJECT;
    } else if (accepted && (fabsf(value) < KNIVES_ZERO_EPSILON_RPM) &&
               debug_fault_code == (float)KNIVES_FAULT_UNARMED_REJECT) {
        debug_fault_code = (float)KNIVES_FAULT_NONE;
    }
    taskEXIT_CRITICAL();
}

static picoros_subscriber_t sub_set_rpm = {
    .topic = {
        .name = "knives/set_rpm",
        .type = "std_msgs::msg::dds_::Float32",
    },
    .user_callback = command_received,
};

static void update_rpm(uint32_t now)
{
    if ((uint32_t)(now - last_rpm_sample_ms) >= RPM_SAMPLE_PERIOD_MS) {
        uint32_t ticks = encoder_ticks;
        uint32_t delta_ticks = ticks - last_encoder_ticks;
        float sample_dt_s = (float)(now - last_rpm_sample_ms) / 1000.0f;
        measured_rpm = ((float)delta_ticks / COUNTS_PER_REVOLUTION) * (60.0f / sample_dt_s);
        if (control_rpm < 0.0f) {
            measured_rpm = -measured_rpm;
        }
        last_encoder_ticks = ticks;
        last_rpm_sample_ms = now;
        new_measurement_available = true;
    }
}

static void update_control(void)
{
    uint32_t now = now_ms();
    taskENTER_CRITICAL();
    float target_rpm = entities_ready ? knives_command_target(&command_state, now) : 0.0f;
    bool authorized = entities_ready && command_state.armed && command_state.received;
    taskEXIT_CRITICAL();

    if (last_control_ms == 0) {
        last_control_ms = now;
        last_rpm_sample_ms = now;
        last_encoder_activity_ms = now;
        return;
    }

    update_rpm(now);

    if (!authorized || (uint32_t)(now - last_control_ms) > MAX_CONTROL_GAP_MS) {
        if (authorized) {
            debug_fault_code = (float)KNIVES_FAULT_GAP_TIMEOUT;
            control_fault();
        } else {
            if (command_state.received && (uint32_t)(now - command_state.received_at_ms) >= KNIVES_COMMAND_TIMEOUT_MS) {
                debug_fault_code = (float)KNIVES_FAULT_CMD_TIMEOUT;
            }
            stop_motor_control();
        }
        last_control_ms = now;
        last_encoder_activity_ms = now;
        return;
    }

    uint32_t activity_ticks = encoder_ticks;
    if (activity_ticks != last_activity_encoder_ticks ||
        fabsf(control_rpm) < BRAKE_THRESHOLD_RPM || fabsf(target_rpm) < 1.0f) {
        last_encoder_activity_ms = now;
        last_activity_encoder_ticks = activity_ticks;
    } else if (!RELOBOT_KNIVES_BENCH &&
               (uint32_t)(now - last_encoder_activity_ms) >= ENCODER_TIMEOUT_MS) {
        debug_fault_code = (float)KNIVES_FAULT_ENCODER_TIMEOUT;
        control_fault();
        last_control_ms = now;
        return;
    }

    float dt_s = (float)(now - last_control_ms) / 1000.0f;
    if (dt_s < ((float)CONTROL_PERIOD_MS / 1000.0f)) {
        return;
    }

    /* Active braking */
    if (fabsf(target_rpm) < 1.0f && !is_braking && fabsf(control_rpm) >= 1.0f &&
        fabsf(measured_rpm) > BRAKE_THRESHOLD_RPM) {
        is_braking = true;
        braking_started_ms = now;
        brake_forward = (control_rpm < 0.0f);
    }

    if (is_braking) {
        if ((uint32_t)(now - braking_started_ms) >= BRAKE_MAX_DURATION_MS) {
            is_braking = false;
            control_rpm = 0.0f;
            pid_controller_reset(&rpm_pid);
            set_motor_pwm(0.0f, true);
            last_control_ms = now;
            return;
        } else if (fabsf(target_rpm) >= 1.0f) {
            is_braking = false;
        } else if (fabsf(measured_rpm) <= BRAKE_THRESHOLD_RPM) {
            is_braking = false;
            control_rpm = 0.0f;
            pid_controller_reset(&rpm_pid);
            set_motor_pwm(0.0f, true);
            last_control_ms = now;
            return;
        } else {
            float brake_pwm = rpm_to_base_pwm(BRAKE_RPM);
            set_motor_pwm(brake_pwm, brake_forward);
            last_control_ms = now;
            return;
        }
    }

    /* Ramping */
    float max_step = RAMP_RATE_RPM_PER_SEC * dt_s;
    float delta = target_rpm - control_rpm;
    delta = pid_clampf(delta, -max_step, max_step);
    control_rpm += delta;

    if (fabsf(control_rpm) < 1.0f) {
        pid_controller_reset(&rpm_pid);
        set_motor_pwm(0.0f, true);
        last_control_ms = now;
        return;
    }

    if (new_measurement_available) {
        last_pwm_adjust = pid_controller_update(&rpm_pid, control_rpm, measured_rpm, dt_s);
        new_measurement_available = false;
    }
    float pwm_adjust = last_pwm_adjust;

    float base_pwm = rpm_to_base_pwm(control_rpm);
    float out_pwm = pid_clampf(base_pwm + pwm_adjust, 0.0f, 1.0f);

    bool new_dir = (control_rpm >= 0.0f);
    if (new_dir != last_dir_forward && fabsf(measured_rpm) > DIRECTION_CHANGE_RPM) {
        set_motor_pwm(0.0f, last_dir_forward);
        last_control_ms = now;
        return;
    }

    set_motor_pwm(out_pwm, new_dir);
    last_control_ms = now;
}

static void control_task(void *argument)
{
    (void)argument;
    TickType_t previous_wake = xTaskGetTickCount();
    for (;;) {
        update_control();
        taskENTER_CRITICAL();
        bool initialized = entities_ready;
        unsigned int stage = diagnostic_stage;
        bool received = command_state.received;
        float current_target = command_state.target_rpm;
        taskEXIT_CRITICAL();

        uint32_t now = now_ms();
        bool active = initialized && received && (fabsf(current_target) >= 1.0f);

        uint8_t r = 0, g = 0, b = 0;
        if ((now % 1000) < 100) {
            /* Heartbeat: brief 100 ms off phase every second (confirms scheduler is running) */
            r = 0; g = 0; b = 0;
        } else if (stage == 0) {
            b = 32; /* Solid Blue: float32_codec_init failed */
        } else if (stage >= 1 && stage <= 5) {
            r = 32; /* Red: waiting/attempting Zenoh session init or retrying declarations */
        } else if (initialized && stage >= 6 && stage <= 10) {
            if (active) {
                g = 32;
            } else {
                r = 24; g = 12;
            }
        } else if (stage >= 6 && stage <= 9) {
            r = 24; g = 20; /* Amber/Yellow: Zenoh session open, declaring ROS entities */
        } else {
            r = 32; b = 16; /* Magenta: unexpected state */
        }

        relobot_board_led_set(r, g, b);
        watchdog_update();
        vTaskDelayUntil(&previous_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void startup_failed(unsigned int stage)
{
    taskENTER_CRITICAL();
    entities_ready = false;
    knives_command_disarm(&command_state);
    taskEXIT_CRITICAL();
    picoros_diagnostic_stage(stage);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool publish_float(picoros_publisher_t *publisher, float value)
{
    uint8_t payload[FLOAT32_CDR_SIZE];
    return float32_codec_encode(value, payload) &&
        picoros_publish(publisher, payload, sizeof(payload)) == PICOROS_OK;
}

static void app_task(void *argument)
{
    (void)argument;

    if (!float32_codec_init()) {
        startup_failed(0);
    }

    sub_set_rpm.topic.rihs_hash = float32_codec_hash();
    pub_current_rpm.topic.rihs_hash = float32_codec_hash();

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
        z_publisher_options_default(&pub_current_rpm.opts);
        if (picoros_publisher_declare(&node, &pub_current_rpm) != PICOROS_OK) {
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        bool debug_ok = true;
        size_t declared_debug = 0;
        for (size_t index = 0; index < sizeof(debug_publishers) / sizeof(debug_publishers[0]); ++index) {
            debug_publishers[index].topic.rihs_hash = float32_codec_hash();
            z_publisher_options_default(&debug_publishers[index].opts);
            if (picoros_publisher_declare(&node, &debug_publishers[index]) != PICOROS_OK) {
                debug_ok = false;
                break;
            }
            declared_debug = index + 1;
        }
        if (!debug_ok) {
            for (size_t index = 0; index < declared_debug; ++index) {
                (void)picoros_publisher_drop(&debug_publishers[index]);
            }
            (void)picoros_publisher_drop(&pub_current_rpm);
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
#if RELOBOT_KNIVES_BENCH
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
            for (size_t index = 0; index < sizeof(debug_publishers) / sizeof(debug_publishers[0]); ++index) {
                (void)picoros_publisher_drop(&debug_publishers[index]);
            }
            (void)picoros_publisher_drop(&pub_current_rpm);
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
#endif
        picoros_diagnostic_stage(9);
        if (picoros_subscriber_declare(&node, &sub_set_rpm) != PICOROS_OK) {
#if RELOBOT_KNIVES_BENCH
            for (size_t index = 0; index < sizeof(bench_publishers) / sizeof(bench_publishers[0]); ++index) {
                (void)picoros_publisher_drop(&bench_publishers[index]);
            }
#endif
            for (size_t index = 0; index < sizeof(debug_publishers) / sizeof(debug_publishers[0]); ++index) {
                (void)picoros_publisher_drop(&debug_publishers[index]);
            }
            (void)picoros_publisher_drop(&pub_current_rpm);
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        taskENTER_CRITICAL();
        entities_ready = true;
        diagnostic_stage = 10;
        taskEXIT_CRITICAL();

#if RELOBOT_KNIVES_BENCH
        uint32_t publish_errors = 0;
        uint32_t last_diagnostics_ms = now_ms();
#endif
        uint32_t last_debug_publish_ms = now_ms();
        uint32_t consecutive_publish_errors = 0;
        while (entities_ready) {
            if (!picoros_interface_is_up()) {
                break;
            }

            taskENTER_CRITICAL();
            float current = measured_rpm;
#if RELOBOT_KNIVES_BENCH
            float target = command_state.target_rpm;
#endif
            taskEXIT_CRITICAL();
            bool published = publish_float(&pub_current_rpm, current);
            if (!published) {
                if (++consecutive_publish_errors >= 5) {
                    break;
                }
            } else {
                consecutive_publish_errors = 0;
            }

            uint32_t now = now_ms();
            if ((uint32_t)(now - last_debug_publish_ms) >= 200) {
                last_debug_publish_ms = now;
                taskENTER_CRITICAL();
                float dbg_target = command_state.target_rpm;
                float dbg_control = control_rpm;
                float dbg_pwm = debug_out_pwm;
                float dbg_ticks = (float)encoder_ticks;
                float dbg_armed = command_state.armed ? 1.0f : 0.0f;
                float dbg_fault = debug_fault_code;
                float dbg_count = (float)debug_cmd_count;
                float dbg_raw = debug_last_raw_cmd;
                float dbg_age = command_state.received ? (float)(now - command_state.received_at_ms) : 99999.0f;
                taskEXIT_CRITICAL();

                float debug_values[] = {
                    dbg_target,
                    dbg_control,
                    dbg_pwm,
                    dbg_ticks,
                    dbg_armed,
                    dbg_fault,
                    dbg_count,
                    dbg_raw,
                    dbg_age,
                };
                _Static_assert(sizeof(debug_values) / sizeof(debug_values[0]) ==
                               sizeof(debug_publishers) / sizeof(debug_publishers[0]),
                               "debug values count must match debug publishers count");
                for (size_t index = 0; index < sizeof(debug_publishers) / sizeof(debug_publishers[0]); ++index) {
                    (void)publish_float(&debug_publishers[index], debug_values[index]);
                }
            }

#if RELOBOT_KNIVES_BENCH
            if (!published) {
                publish_errors++;
            }
            if ((uint32_t)(now - last_diagnostics_ms) >= 1000) {
                float values[] = {
                    target,
                    (float)xPortGetFreeHeapSize(),
                    (float)xPortGetMinimumEverFreeHeapSize(),
                    (float)uxTaskGetStackHighWaterMark(NULL),
                    (float)uxTaskGetStackHighWaterMark(control_task_handle),
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
            vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        }

        /* Session or interface dropped: clean up and retry initialization */
        taskENTER_CRITICAL();
        entities_ready = false;
        knives_command_disarm(&command_state);
        taskEXIT_CRITICAL();
        stop_motor_control();

        (void)picoros_subscriber_drop(&sub_set_rpm);
        (void)picoros_publisher_drop(&pub_current_rpm);
#if RELOBOT_KNIVES_BENCH
        for (size_t index = 0; index < sizeof(bench_publishers) / sizeof(bench_publishers[0]); ++index) {
            (void)picoros_publisher_drop(&bench_publishers[index]);
        }
#endif
        for (size_t index = 0; index < sizeof(debug_publishers) / sizeof(debug_publishers[0]); ++index) {
            (void)picoros_publisher_drop(&debug_publishers[index]);
        }
        (void)picoros_node_drop(&node);
        picoros_interface_close();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void init_motor_io(void)
{
#if !RELOBOT_KNIVES_BENCH
    gpio_init(PIN_FWD_EN);
    gpio_set_dir(PIN_FWD_EN, GPIO_OUT);
    gpio_put(PIN_FWD_EN, 0);

    gpio_init(PIN_REV_EN);
    gpio_set_dir(PIN_REV_EN, GPIO_OUT);
    gpio_put(PIN_REV_EN, 0);

    gpio_set_function(PIN_FWD_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_REV_PWM, GPIO_FUNC_PWM);

    uint slice_fwd = pwm_gpio_to_slice_num(PIN_FWD_PWM);
    uint slice_rev = pwm_gpio_to_slice_num(PIN_REV_PWM);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    pwm_init(slice_fwd, &cfg, true);
    pwm_init(slice_rev, &cfg, true);

    pwm_set_gpio_level(PIN_FWD_PWM, 0);
    pwm_set_gpio_level(PIN_REV_PWM, 0);

    gpio_init(PIN_ENCODER);
    gpio_set_dir(PIN_ENCODER, GPIO_IN);
    gpio_pull_up(PIN_ENCODER);
    gpio_set_irq_enabled_with_callback(PIN_ENCODER, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);
#endif
}

int main(void)
{
    init_motor_io();
    emergency_stop();
    /* Nothing feeds this watchdog until control_task's first loop iteration,
     * so everything between here and vTaskStartScheduler() must stay well
     * under WATCHDOG_TIMEOUT_MS and must not block (e.g. no stdio_usb). */
    watchdog_enable(WATCHDOG_TIMEOUT_MS, false);
    relobot_board_init();

    pid_controller_init(
        &rpm_pid,
        PI_KP,
        PI_KI,
        0.0f,
        -PI_OUTPUT_LIMIT,
        PI_OUTPUT_LIMIT,
        -PI_INTEGRAL_LIMIT,
        PI_INTEGRAL_LIMIT);
    emergency_stop();

    if (xTaskCreate(control_task, "motor_control", CONTROL_TASK_STACK_WORDS,
                    NULL, CONTROL_TASK_PRIORITY, &control_task_handle) != pdPASS ||
        xTaskCreate(app_task, "picoros_app", APP_TASK_STACK_WORDS,
                    NULL, APP_TASK_PRIORITY, &app_task_handle) != pdPASS) {
        emergency_stop();
        for (;;) { tight_loop_contents(); }
    }

    vTaskStartScheduler();

    emergency_stop();
    for (;;) { tight_loop_contents(); }
}
