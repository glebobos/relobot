#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pid_controller.h"
#include "pico_uart_transports.h"

#define PWM_WRAP                    12499
#define COMMAND_TIMEOUT_MS          5000
#define WATCHDOG_TIMEOUT_MS         2000
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
#define LED_PIN                     PICO_DEFAULT_LED_PIN

#define AGENT_POLL_MS               500
#define AGENT_KEEPALIVE_MS          1000
#define AGENT_PING_TIMEOUT_MS       100
#define AGENT_PING_ATTEMPTS         1
#define TIME_SYNC_TIMEOUT_MS        1000

#define DIRECTION_DEADTIME_MS       2
#define DIRECTION_CHANGE_RPM        50.0f

#define EXECUTE_EVERY_N_MS(MS, X) do {                          \
    static uint64_t init = 0;                                   \
    if (init == 0) { init = now_ms(); }                         \
    if (now_ms() - init > (MS)) {                               \
        X;                                                      \
        init = now_ms();                                        \
    }                                                           \
} while (0)

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

typedef struct {
    rcl_subscription_t sub_set_rpm;
    rcl_publisher_t pub_current_rpm;
    rcl_timer_t telemetry_timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;
} app_context_t;

static app_context_t ctx;
static std_msgs__msg__Float32 rpm_cmd_msg;
static std_msgs__msg__Float32 rpm_telemetry_msg;

static volatile uint32_t encoder_ticks = 0;

static float target_rpm = 0.0f;
static float control_rpm = 0.0f;
static float measured_rpm = 0.0f;
static pid_controller_t rpm_pid;

static uint64_t last_command_ms = 0;
static uint64_t last_control_ms = 0;
static uint64_t last_rpm_sample_ms = 0;
static uint32_t last_encoder_ticks = 0;

static bool is_braking = false;
static bool brake_forward = true;    /* direction to apply brake pulse (#1) */
static bool last_dir_forward = true; /* last direction applied to H-bridge (#2/#7) */
static bool new_measurement_available = false; /* gates PID integrator (#5/#P6) */
static float last_pwm_adjust = 0.0f;           /* reused on ticks with stale data (#5) */

static enum states state;

static inline uint64_t now_ms(void)
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

    /* #2/#7 — enforce dead-time on direction change to prevent BTS7960 cross-conduction */
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

static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == PIN_ENCODER) {
        encoder_ticks++;
    }
}

static void command_callback(const void *msg_in)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    target_rpm = clampf(msg->data, -MAX_ABS_RPM, MAX_ABS_RPM);
    last_command_ms = now_ms();
}

static void telemetry_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    rpm_telemetry_msg.data = measured_rpm;
    rcl_ret_t pub_ret = rcl_publish(&ctx.pub_current_rpm, &rpm_telemetry_msg, NULL);
    (void)pub_ret;
}

static void update_control(void)
{
    uint64_t now = now_ms();

    if (last_control_ms == 0) {
        last_control_ms = now;
        last_rpm_sample_ms = now;
        last_command_ms = now;
        return;
    }

    if ((now - last_command_ms) > COMMAND_TIMEOUT_MS) {
        target_rpm = 0.0f;
    }

    // --- RPM measurement on a long window (600 ms) to accumulate ticks ---
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
        new_measurement_available = true;  /* #5/#P6 — signal fresh data to PID */
    }

    // --- control loop runs at CONTROL_PERIOD_MS ---
    float dt_s = (float)(now - last_control_ms) / 1000.0f;
    if (dt_s < (CONTROL_PERIOD_MS / 1000.0f)) {
        return;
    }

    // --- active braking: when target≈0 and motor is still spinning ---
    if (fabsf(target_rpm) < 1.0f && !is_braking && fabsf(measured_rpm) > BRAKE_THRESHOLD_RPM) {
        is_braking = true;
        /* #1 — brake direction is opposite of current motion direction */
        brake_forward = (control_rpm < 0.0f);
    }

    if (is_braking) {
        /* #3 — abort braking immediately if a new non-zero command arrives */
        if (fabsf(target_rpm) >= 1.0f) {
            is_braking = false;
            /* fall through to normal control path below */
        } else if (fabsf(measured_rpm) <= BRAKE_THRESHOLD_RPM) {
            // braking complete — hard stop
            is_braking = false;
            control_rpm = 0.0f;
            pid_controller_reset(&rpm_pid);
            set_motor_pwm(0.0f, true);
            last_control_ms = now;
            return;
        } else {
            // apply opposing pulse to decelerate
            float brake_pwm = rpm_to_base_pwm(BRAKE_RPM);
            set_motor_pwm(brake_pwm, brake_forward);  /* #1 — correct brake direction */
            last_control_ms = now;
            return;
        }
    }

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

    /* #5/#P6 — only update PID integrator when fresh measurement data is available */
    if (new_measurement_available) {
        last_pwm_adjust = pid_controller_update(&rpm_pid, control_rpm, measured_rpm, dt_s);
        new_measurement_available = false;
    }
    float pwm_adjust = last_pwm_adjust;

    float base_pwm = rpm_to_base_pwm(control_rpm);
    float out_pwm = clampf(base_pwm + pwm_adjust, 0.0f, 1.0f);

    /* #4 — coast until measured speed is low enough to safely switch direction */
    bool new_dir = (control_rpm >= 0.0f);
    if (new_dir != last_dir_forward && fabsf(measured_rpm) > DIRECTION_CHANGE_RPM) {
        set_motor_pwm(0.0f, last_dir_forward);
        last_control_ms = now;
        return;
    }

    set_motor_pwm(out_pwm, new_dir);

    last_control_ms = now;
}

static void set_state_led(enum states s)
{
    gpio_put(LED_PIN, s == AGENT_CONNECTED);
}

static bool create_entities(void)
{
    ctx.allocator = rcl_get_default_allocator();

    if (rclc_support_init(&ctx.support, 0, NULL, &ctx.allocator) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&ctx.node, "knives_node", "", &ctx.support) != RCL_RET_OK)
        goto fail_support;

    if (rclc_subscription_init_default(
            &ctx.sub_set_rpm,
            &ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "knives/set_rpm") != RCL_RET_OK)
        goto fail_node;

    if (rclc_publisher_init_best_effort(
            &ctx.pub_current_rpm,
            &ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "knives/current_rpm") != RCL_RET_OK)
        goto fail_sub;

    if (rclc_timer_init_default(
            &ctx.telemetry_timer,
            &ctx.support,
            RCL_MS_TO_NS(TELEMETRY_PERIOD_MS),
            telemetry_timer_callback) != RCL_RET_OK)
        goto fail_pub;

    ctx.executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&ctx.executor, &ctx.support.context, 2, &ctx.allocator) != RCL_RET_OK)
        goto fail_timer;

    if (rclc_executor_add_subscription(&ctx.executor, &ctx.sub_set_rpm, &rpm_cmd_msg, &command_callback, ON_NEW_DATA) != RCL_RET_OK)
        goto fail_executor;

    if (rclc_executor_add_timer(&ctx.executor, &ctx.telemetry_timer) != RCL_RET_OK)
        goto fail_executor;

    return true;

fail_executor:
    rclc_executor_fini(&ctx.executor);
fail_timer:
    {
        rcl_ret_t timer_ret = rcl_timer_fini(&ctx.telemetry_timer);
        (void)timer_ret;
    }
fail_pub:
    {
        rcl_ret_t pub_ret = rcl_publisher_fini(&ctx.pub_current_rpm, &ctx.node);
        (void)pub_ret;
    }
fail_sub:
    {
        rcl_ret_t sub_ret = rcl_subscription_fini(&ctx.sub_set_rpm, &ctx.node);
        (void)sub_ret;
    }
fail_node:
    {
        rcl_ret_t node_ret = rcl_node_fini(&ctx.node);
        (void)node_ret;
    }
fail_support:
    rclc_support_fini(&ctx.support);
    return false;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&ctx.support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_ret_t sub_ret   = rcl_subscription_fini(&ctx.sub_set_rpm, &ctx.node);
    rcl_ret_t pub_ret   = rcl_publisher_fini(&ctx.pub_current_rpm, &ctx.node);
    rcl_ret_t timer_ret = rcl_timer_fini(&ctx.telemetry_timer);
    rclc_executor_fini(&ctx.executor);
    rcl_ret_t node_ret  = rcl_node_fini(&ctx.node);
    rclc_support_fini(&ctx.support);

    (void)sub_ret;
    (void)pub_ret;
    (void)timer_ret;
    (void)node_ret;
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
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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

    state = WAITING_AGENT;

    while (true) {
        watchdog_update();
        set_state_led(state);

        switch (state) {
            case WAITING_AGENT:
                emergency_stop();
                EXECUTE_EVERY_N_MS(AGENT_POLL_MS,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                        ? AGENT_AVAILABLE : WAITING_AGENT;
                );
                break;

            case AGENT_AVAILABLE:
                if (create_entities()) {
                    rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
                    /* #10 — enable watchdog only after connection; avoids firing during slow init */
                    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
                    state = AGENT_CONNECTED;
                    last_command_ms = now_ms();
                } else {
                    emergency_stop();
                    state = WAITING_AGENT;
                }
                break;

            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(AGENT_KEEPALIVE_MS,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                        ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                );

                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&ctx.executor, RCL_MS_TO_NS(10));
                    update_control();
                    sleep_ms(1);
                }
                break;

            case AGENT_DISCONNECTED:
                emergency_stop();
                destroy_entities();
                state = WAITING_AGENT;
                break;

            default:
                emergency_stop();
                state = WAITING_AGENT;
                break;
        }
    }
}
