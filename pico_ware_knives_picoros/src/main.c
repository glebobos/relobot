#include <math.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <picoros.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "tusb.h"

#include "float32_codec.h"
#include "pid_controller.h"
#include "relobot/board.h"
#include "relobot/picoros_runtime.h"

#define PWM_WRAP                    12499
#define COMMAND_TIMEOUT_MS          5000
#define CONTROL_PERIOD_MS           20
#define RPM_SAMPLE_PERIOD_MS        600
#define TELEMETRY_PERIOD_MS         100

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
#define MAX_ABS_RPM                 3500.0f

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

static float target_rpm = 0.0f;
static float control_rpm = 0.0f;
static float measured_rpm = 0.0f;
static pid_controller_t rpm_pid;

static uint32_t last_command_ms = 0;
static uint32_t last_control_ms = 0;
static uint32_t last_rpm_sample_ms = 0;
static uint32_t last_encoder_ticks = 0;

static bool is_braking = false;
static bool brake_forward = true;
static bool last_dir_forward = true;
static bool new_measurement_available = false;
static float last_pwm_adjust = 0.0f;
static bool ros_connected = false;

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

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static inline float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float rpm_to_base_pwm(float rpm)
{
    float omega = fabsf(rpm) * (2.0f * (float)M_PI / 60.0f);
    return clampf(FEED_FORWARD_K * omega + FEED_FORWARD_C, 0.0f, 1.0f);
}

static void set_motor_pwm(float pwm, bool forward)
{
    uint16_t level = (uint16_t)(clampf(pwm, 0.0f, 1.0f) * (float)PWM_WRAP);

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
}

static void emergency_stop(void)
{
    target_rpm = 0.0f;
    control_rpm = 0.0f;
    is_braking = false;
    pid_controller_reset(&rpm_pid);
    set_motor_pwm(0.0f, true);
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
    (void)itf;
    if (p_line_coding && p_line_coding->bit_rate == 1200) {
        emergency_stop();
        reset_usb_boot(0, 0);
    }
}

static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == PIN_ENCODER) {
        encoder_ticks++;
    }
}

static void command_received(uint8_t *payload, size_t length)
{
    float value;
    if (float32_codec_decode(payload, length, &value)) {
        taskENTER_CRITICAL();
        target_rpm = clampf(value, -MAX_ABS_RPM, MAX_ABS_RPM);
        last_command_ms = now_ms();
        taskEXIT_CRITICAL();
    }
}

static picoros_subscriber_t sub_set_rpm = {
    .topic = {
        .name = "knives/set_rpm",
        .type = "std_msgs::msg::dds_::Float32",
    },
    .user_callback = command_received,
};

static void update_control(void)
{
    uint32_t now = now_ms();

    if (last_control_ms == 0) {
        last_control_ms = now;
        last_rpm_sample_ms = now;
        last_command_ms = now;
        return;
    }

    /* Enforce 5000 ms local fail-safe command timeout and disconnection stop */
    if (!ros_connected || (now - last_command_ms) > COMMAND_TIMEOUT_MS) {
        target_rpm = 0.0f;
    }

    /* RPM measurement on 600 ms window */
    if ((now - last_rpm_sample_ms) >= RPM_SAMPLE_PERIOD_MS) {
        uint32_t ticks = encoder_ticks;
        uint32_t delta_ticks = ticks - last_encoder_ticks;
        float sample_dt_s = (float)(now - last_rpm_sample_ms) / 1000.0f;
        if (sample_dt_s > 0.0f) {
            measured_rpm = ((float)delta_ticks / COUNTS_PER_REVOLUTION) * (60.0f / sample_dt_s);
            if (control_rpm < 0.0f) {
                measured_rpm = -measured_rpm;
            }
        }
        last_encoder_ticks = ticks;
        last_rpm_sample_ms = now;
        new_measurement_available = true;
    }

    float dt_s = (float)(now - last_control_ms) / 1000.0f;
    if (dt_s < ((float)CONTROL_PERIOD_MS / 1000.0f)) {
        return;
    }

    /* Active braking */
    if (fabsf(target_rpm) < 1.0f && !is_braking && fabsf(measured_rpm) > BRAKE_THRESHOLD_RPM) {
        is_braking = true;
        brake_forward = (control_rpm < 0.0f);
    }

    if (is_braking) {
        if (fabsf(target_rpm) >= 1.0f) {
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
    delta = clampf(delta, -max_step, max_step);
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
    float out_pwm = clampf(base_pwm + pwm_adjust, 0.0f, 1.0f);

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
        bool connected = ros_connected;
        unsigned int stage = diagnostic_stage;
        uint32_t last_cmd = last_command_ms;
        float current_target = target_rpm;
        taskEXIT_CRITICAL();

        uint32_t now = now_ms();
        bool active = connected && ((now - last_cmd) < 1500) && (fabsf(current_target) >= 1.0f);

        uint8_t r = 0, g = 0, b = 0;
        if ((now % 1000) < 100) {
            /* Heartbeat: brief 100 ms off phase every second (confirms scheduler is running) */
            r = 0; g = 0; b = 0;
        } else if (stage == 0) {
            b = 32; /* Solid Blue: float32_codec_init failed */
        } else if (stage >= 1 && stage <= 5) {
            r = 32; /* Red: waiting/attempting Zenoh session init or retrying declarations */
        } else if (stage >= 6 && stage <= 9) {
            r = 24; g = 20; /* Amber/Yellow: Zenoh session open, declaring ROS entities */
        } else if (stage == 10) {
            if (!connected) {
                r = 32; /* Red: Disconnected / router down / publish error */
            } else if (active) {
                g = 32; /* Green: ROS topics communicating & active target RPM running */
            } else {
                r = 24; g = 12; /* Amber/Orange: ROS topics connected & healthy, standing by idle */
            }
        } else {
            r = 32; b = 16; /* Magenta: unexpected state */
        }

        relobot_board_led_set(r, g, b);
        vTaskDelayUntil(&previous_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void app_task(void *argument)
{
    (void)argument;

    if (!float32_codec_init()) {
        picoros_diagnostic_stage(0);
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    sub_set_rpm.topic.rihs_hash = float32_codec_hash();
    pub_current_rpm.topic.rihs_hash = float32_codec_hash();

    for (;;) {
        picoros_diagnostic_stage(1);
        taskENTER_CRITICAL();
        ros_connected = false;
        taskEXIT_CRITICAL();

        picoros_res_t status = relobot_picoros_init();
        if (status != PICOROS_OK) {
            picoros_diagnostic_stage(3);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        picoros_diagnostic_stage(7);
        if (picoros_node_init(&node) != PICOROS_OK) {
            picoros_diagnostic_stage(4);
            picoros_interface_close();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        picoros_diagnostic_stage(8);
        z_publisher_options_default(&pub_current_rpm.opts);
        if (picoros_publisher_declare(&node, &pub_current_rpm) != PICOROS_OK) {
            picoros_diagnostic_stage(5);
            picoros_node_drop(&node);
            picoros_interface_close();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        picoros_diagnostic_stage(9);
        if (picoros_subscriber_declare(&node, &sub_set_rpm) != PICOROS_OK) {
            picoros_diagnostic_stage(5);
            picoros_publisher_drop(&pub_current_rpm);
            picoros_node_drop(&node);
            picoros_interface_close();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        taskENTER_CRITICAL();
        ros_connected = true;
        diagnostic_stage = 10;
        last_command_ms = now_ms();
        taskEXIT_CRITICAL();

        uint32_t consecutive_publish_errors = 0;
        while (ros_connected) {
            taskENTER_CRITICAL();
            float current = measured_rpm;
            taskEXIT_CRITICAL();

            uint8_t payload[FLOAT32_CDR_SIZE];
            if (float32_codec_encode(current, payload)) {
                picoros_res_t pub_res = picoros_publish(&pub_current_rpm, payload, sizeof(payload));
                taskENTER_CRITICAL();
                if (pub_res == PICOROS_OK) {
                    consecutive_publish_errors = 0;
                } else {
                    if (++consecutive_publish_errors >= 5) {
                        ros_connected = false;
                    }
                }
                taskEXIT_CRITICAL();
            }

            vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        }

        /* Session/link disconnected: clean up entities and retry initialization */
        picoros_subscriber_drop(&sub_set_rpm);
        picoros_publisher_drop(&pub_current_rpm);
        picoros_node_drop(&node);
        picoros_interface_close();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void init_motor_io(void)
{
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
}

int main(void)
{
    relobot_board_init();
    init_motor_io();

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
