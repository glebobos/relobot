#include <math.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico_uart_transports.h"

// --- Timing ---
#define PWM_WRAP                    6249U       // ~20 kHz at 125 MHz sys_clk (silent)
#define COMMAND_TIMEOUT_MS          2000U
#define WATCHDOG_TIMEOUT_MS         2000U
#define CONTROL_PERIOD_MS           10U         // 100 Hz
#define PUBLISH_PERIOD_MS           33U         // ~30 Hz

// --- Physical constants ---
#define COUNTS_PER_REVOLUTION       60.0f
#define TWO_PI                      (2.0f * (float)M_PI)
#define VELOCITY_EMA_ALPHA          0.25f

// --- Control limits ---
#define MAX_PWM_COMMAND             1.0f        // absolute clamp on raw PWM commands
#define STALL_DETECT_CYCLES         50

// --- Pin assignments (XIAO RP2040 board labels -> GPIO) ---
#define PIN_LEFT_PWM                27          // D1, Slice 5 Ch B
#define PIN_LEFT_DIR                26          // D0
#define PIN_RIGHT_PWM               28          // D2, Slice 6 Ch A
#define PIN_RIGHT_DIR               7           // D5
#define PIN_ENCODER_LEFT            6           // D4
#define PIN_ENCODER_RIGHT           3           // D10
#define LED_PIN                     PICO_DEFAULT_LED_PIN

#define DIRECTION_DEADTIME_MS       2U

// --- Agent polling ---
#define AGENT_POLL_MS               500U
#define AGENT_KEEPALIVE_MS          1000U
#define AGENT_PING_TIMEOUT_MS       100U
#define AGENT_PING_ATTEMPTS         1
#define TIME_SYNC_TIMEOUT_MS        1000U
#define TIME_SYNC_PERIOD_MS         10000U      // re-sync ROS clock every 10 s

#define EXECUTE_EVERY_N_MS(MS, X)  do {                             \
    static uint64_t _ts = 0;                                        \
    if (_ts == 0) { _ts = now_ms(); }                               \
    if (now_ms() - _ts >= (MS)) { X; _ts += (MS); }                \
} while (0)

// ---------------------------------------------------------------------------

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

typedef struct {
    uint              pwm_pin;
    uint              dir_pin;
    uint              enc_pin;
    volatile uint32_t encoder_ticks;
    volatile uint32_t last_tick_us;    // µs timestamp of most recent tick (ISR-written)
    volatile uint32_t tick_period_us;  // µs between last two ticks  (ISR-written)
    uint32_t          last_enc_ticks;
    float             cmd_vel;          // direct PWM command (-1.0 to 1.0)
    float             measured_vel;     // rad/s, EMA of period-based measurements
    int64_t           tick_accumulator; // signed tick count for drift-free position
    bool              last_dir_fwd;
    uint64_t          dir_change_at_ms; // wall-clock time of last direction change
    uint32_t          encoder_age;      // control cycles since last encoder tick
} motor_t;

typedef struct {
    rcl_subscription_t sub_commands;
    rcl_publisher_t    pub_states;
    rcl_timer_t        publish_timer;
    rcl_node_t         node;
    rcl_allocator_t    allocator;
    rclc_support_t     support;
    rclc_executor_t    executor;
} app_context_t;

// ---------------------------------------------------------------------------
// Static storage

static app_context_t ctx;

// Publisher message (state)
static sensor_msgs__msg__JointState state_msg;
static rosidl_runtime_c__String     state_name_arr[2];
static double                        state_pos_arr[2];
static double                        state_vel_arr[2];
static double                        state_eff_arr[2];
static char                          state_name0_buf[] = "left_wheel_joint";
static char                          state_name1_buf[] = "right_wheel_joint";
static char                          state_frame_id_buf[12];

// Subscriber message (command) — pre-allocated for 2 joints
static sensor_msgs__msg__JointState cmd_msg;
static rosidl_runtime_c__String     cmd_name_arr[2];
static double                        cmd_vel_arr[2];
static double                        cmd_pos_arr[2];
static double                        cmd_eff_arr[2];
static char                          cmd_name0_buf[32];
static char                          cmd_name1_buf[32];
static char                          cmd_frame_id_buf[32];

static motor_t left_motor;
static motor_t right_motor;

static uint64_t last_command_ms = 0;
static uint64_t last_control_ms = 0;

static enum states state;

// ---------------------------------------------------------------------------

static inline uint64_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void set_motor_pwm(motor_t *m, float pwm_fraction, bool fwd)
{
    uint16_t level = (uint16_t)(clampf(pwm_fraction, 0.0f, 1.0f) * (float)PWM_WRAP);

    if (level == 0) {
        pwm_set_gpio_level(m->pwm_pin, 0);
        gpio_put(m->dir_pin, 0);
        m->last_dir_fwd = fwd;
        return;
    }

    if (fwd != m->last_dir_fwd) {
        // Direction change: coast outputs and record timestamp — do NOT block.
        pwm_set_gpio_level(m->pwm_pin, 0);
        gpio_put(m->dir_pin, 0);
        m->dir_change_at_ms = now_ms();
        m->last_dir_fwd     = fwd;
        return;  // coast this cycle; PWM applied once deadtime has elapsed
    }

    // Still within deadtime window — keep coasting.
    if ((now_ms() - m->dir_change_at_ms) < DIRECTION_DEADTIME_MS) {
        return;
    }

    if (m->dir_pin == PIN_RIGHT_DIR) {
        gpio_put(m->dir_pin, fwd ? 1 : 0);
    } else {
        gpio_put(m->dir_pin, fwd ? 0 : 1);
    }
    pwm_set_gpio_level(m->pwm_pin, level);
}

static void stop_motor(motor_t *m)
{
    pwm_set_gpio_level(m->pwm_pin, 0);
    gpio_put(m->dir_pin, 0);
    m->encoder_age  = 0;     // reset so re-start doesn't immediately trigger stall
}

static void emergency_stop_all(void)
{
    stop_motor(&left_motor);
    stop_motor(&right_motor);
    left_motor.cmd_vel    = 0.0f;
    right_motor.cmd_vel   = 0.0f;
}

static void encoder_isr(uint gpio, uint32_t events)
{
    (void)events;
    uint32_t now_us = time_us_32();
    if (gpio == PIN_ENCODER_LEFT) {
        uint32_t diff = now_us - left_motor.last_tick_us;
        if (diff > 7000U) {  // 7 ms debounce to filter noise/chatter
            left_motor.tick_period_us = diff;
            left_motor.last_tick_us   = now_us;
            left_motor.encoder_ticks++;
        }
    } else if (gpio == PIN_ENCODER_RIGHT) {
        uint32_t diff = now_us - right_motor.last_tick_us;
        if (diff > 7000U) {  // 7 ms debounce to filter noise/chatter
            right_motor.tick_period_us = diff;
            right_motor.last_tick_us   = now_us;
            right_motor.encoder_ticks++;
        }
    }
}

static void command_callback(const void *msg_in)
{
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msg_in;
    if (msg->velocity.size < 2) {
        return;
    }

    // Match by joint name so motor assignment is order-independent.
    float new_left  = left_motor.cmd_vel;
    float new_right = right_motor.cmd_vel;
    bool  got_left  = false;
    bool  got_right = false;

    for (size_t i = 0; i < msg->name.size && i < msg->velocity.size; i++) {
        const char *name = msg->name.data[i].data;
        if (name && strncmp(name, "left_wheel_joint",  msg->name.data[i].size) == 0) {
            new_left = clampf((float)msg->velocity.data[i], -MAX_PWM_COMMAND, MAX_PWM_COMMAND);
            got_left = true;
        } else if (name && strncmp(name, "right_wheel_joint", msg->name.data[i].size) == 0) {
            new_right = clampf((float)msg->velocity.data[i], -MAX_PWM_COMMAND, MAX_PWM_COMMAND);
            got_right = true;
        }
    }

    // Fall back to positional mapping if names are absent (e.g. testing with ros2 topic pub).
    if (!got_left && !got_right) {
        new_left  = clampf((float)msg->velocity.data[0], -MAX_PWM_COMMAND, MAX_PWM_COMMAND);
        new_right = clampf((float)msg->velocity.data[1], -MAX_PWM_COMMAND, MAX_PWM_COMMAND);
    }

    left_motor.cmd_vel  = new_left;
    right_motor.cmd_vel = new_right;
    last_command_ms = now_ms();
}

static void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    int64_t now_ns = rmw_uros_epoch_nanos();
    state_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    state_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    state_pos_arr[0] = (double)left_motor.tick_accumulator  * ((double)TWO_PI / (double)COUNTS_PER_REVOLUTION);
    state_pos_arr[1] = (double)right_motor.tick_accumulator * ((double)TWO_PI / (double)COUNTS_PER_REVOLUTION);
    state_vel_arr[0] = (double)left_motor.measured_vel;
    state_vel_arr[1] = (double)right_motor.measured_vel;

    rcl_publish(&ctx.pub_states, &state_msg, NULL);
}

static void update_motor(motor_t *m, float dt_s)
{
    if (dt_s <= 0.0f || !isfinite(dt_s)) {
        return;
    }

    // --- Encoder delta ---
    uint32_t ticks = m->encoder_ticks;
    uint32_t delta = ticks - m->last_enc_ticks;
    m->last_enc_ticks = ticks;

    // --- Velocity and position ---
    if (delta > 0) {
        uint32_t period_us = m->tick_period_us;
        if (period_us > 500U && period_us < 5000000U) {  // 500 µs .. 5 s bounds
            float direction = m->last_dir_fwd ? 1.0f : -1.0f;
            float raw_vel   = direction * TWO_PI
                              / (COUNTS_PER_REVOLUTION * ((float)period_us * 1e-6f));
            if (isfinite(raw_vel)) {
                m->measured_vel = VELOCITY_EMA_ALPHA * raw_vel
                                + (1.0f - VELOCITY_EMA_ALPHA) * m->measured_vel;
            }
        }
        m->tick_accumulator += m->last_dir_fwd ? (int64_t)delta : -(int64_t)delta;
        m->encoder_age  = 0;
    } else {
        m->encoder_age++;
        if (m->encoder_age > STALL_DETECT_CYCLES) {
            m->measured_vel *= (1.0f - VELOCITY_EMA_ALPHA);
        }
    }

    // --- Calibration: directly apply command as PWM fraction ---
    float out_pwm = m->cmd_vel; // direct PWM command (from -1.0 to 1.0)
    bool  fwd     = (out_pwm >= 0.0f);
    float abs_pwm = fabsf(out_pwm);

    set_motor_pwm(m, abs_pwm, fwd);
}

static void update_control(void)
{
    uint64_t now = now_ms();

    if (last_control_ms == 0) {
        last_control_ms = now;
        last_command_ms = now;
        return;
    }

    if ((now - last_command_ms) > COMMAND_TIMEOUT_MS) {
        left_motor.cmd_vel  = 0.0f;
        right_motor.cmd_vel = 0.0f;
    }

    float dt_s = (float)(now - last_control_ms) / 1000.0f;
    if (dt_s < ((float)CONTROL_PERIOD_MS / 1000.0f)) {
        return;
    }

    update_motor(&left_motor,  dt_s);
    update_motor(&right_motor, dt_s);

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

    if (rclc_node_init_default(&ctx.node, "wheels_calibration_node", "", &ctx.support) != RCL_RET_OK)
        goto fail_support;

    if (rclc_subscription_init_default(
            &ctx.sub_commands,
            &ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/robot_joint_commands") != RCL_RET_OK)
        goto fail_node;

    if (rclc_publisher_init_best_effort(
            &ctx.pub_states,
            &ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/robot_joint_states") != RCL_RET_OK)
        goto fail_sub;

    if (rclc_timer_init_default(
            &ctx.publish_timer,
            &ctx.support,
            RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
            publish_timer_callback) != RCL_RET_OK)
        goto fail_pub;

    ctx.executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&ctx.executor, &ctx.support.context, 2, &ctx.allocator) != RCL_RET_OK)
        goto fail_timer;

    if (rclc_executor_add_subscription(
            &ctx.executor, &ctx.sub_commands, &cmd_msg,
            &command_callback, ON_NEW_DATA) != RCL_RET_OK)
        goto fail_executor;

    if (rclc_executor_add_timer(&ctx.executor, &ctx.publish_timer) != RCL_RET_OK)
        goto fail_executor;

    return true;

fail_executor:
    rclc_executor_fini(&ctx.executor);
fail_timer:
    { rcl_ret_t r = rcl_timer_fini(&ctx.publish_timer); (void)r; }
fail_pub:
    { rcl_ret_t r = rcl_publisher_fini(&ctx.pub_states, &ctx.node); (void)r; }
fail_sub:
    { rcl_ret_t r = rcl_subscription_fini(&ctx.sub_commands, &ctx.node); (void)r; }
fail_node:
    { rcl_ret_t r = rcl_node_fini(&ctx.node); (void)r; }
fail_support:
    rclc_support_fini(&ctx.support);
    return false;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&ctx.support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_ret_t r1 = rcl_subscription_fini(&ctx.sub_commands, &ctx.node);
    rcl_ret_t r2 = rcl_publisher_fini(&ctx.pub_states, &ctx.node);
    rcl_ret_t r3 = rcl_timer_fini(&ctx.publish_timer);
    rclc_executor_fini(&ctx.executor);
    rcl_ret_t r4 = rcl_node_fini(&ctx.node);
    rclc_support_fini(&ctx.support);

    (void)r1; (void)r2; (void)r3; (void)r4;
}

static void init_motor_pins(uint pwm_pin, uint dir_pin)
{
    // Initialize PWM pin
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pwm_pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pwm_pin, 0);

    // Initialize DIR pin as GPIO output
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, 0);
}

static void init_encoder_pin(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static void init_motor_struct(motor_t *m, uint pwm_pin, uint dir_pin, uint enc_pin)
{
    memset(m, 0, sizeof(*m));
    m->pwm_pin      = pwm_pin;
    m->dir_pin      = dir_pin;
    m->enc_pin      = enc_pin;
    m->last_dir_fwd = true;
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

    // --- Motor initialisation ---
    init_motor_struct(&left_motor,  PIN_LEFT_PWM,  PIN_LEFT_DIR,  PIN_ENCODER_LEFT);
    init_motor_struct(&right_motor, PIN_RIGHT_PWM, PIN_RIGHT_DIR, PIN_ENCODER_RIGHT);

    init_motor_pins(PIN_LEFT_PWM,  PIN_LEFT_DIR);
    init_motor_pins(PIN_RIGHT_PWM, PIN_RIGHT_DIR);

    init_encoder_pin(PIN_ENCODER_LEFT);
    init_encoder_pin(PIN_ENCODER_RIGHT);

    // Single ISR handles both encoder pins
    gpio_set_irq_enabled_with_callback(PIN_ENCODER_LEFT,  GPIO_IRQ_EDGE_RISE, true, &encoder_isr);
    gpio_set_irq_enabled(PIN_ENCODER_RIGHT, GPIO_IRQ_EDGE_RISE, true);

    // --- Publisher message pre-allocation ---
    state_name_arr[0].data     = state_name0_buf;
    state_name_arr[0].size     = sizeof(state_name0_buf) - 1;
    state_name_arr[0].capacity = sizeof(state_name0_buf);
    state_name_arr[1].data     = state_name1_buf;
    state_name_arr[1].size     = sizeof(state_name1_buf) - 1;
    state_name_arr[1].capacity = sizeof(state_name1_buf);
    state_msg.name.data        = state_name_arr;
    state_msg.name.size        = 2;
    state_msg.name.capacity    = 2;

    state_msg.position.data     = state_pos_arr;
    state_msg.position.size     = 2;
    state_msg.position.capacity = 2;

    state_msg.velocity.data     = state_vel_arr;
    state_msg.velocity.size     = 2;
    state_msg.velocity.capacity = 2;

    state_msg.effort.data     = state_eff_arr;
    state_msg.effort.size     = 2;
    state_msg.effort.capacity = 2;

    state_msg.header.frame_id.data     = state_frame_id_buf;
    state_msg.header.frame_id.size     = 0;
    state_msg.header.frame_id.capacity = sizeof(state_frame_id_buf);

    // --- Subscriber message pre-allocation ---
    cmd_name_arr[0].data     = cmd_name0_buf;
    cmd_name_arr[0].size     = 0;
    cmd_name_arr[0].capacity = sizeof(cmd_name0_buf);
    cmd_name_arr[1].data     = cmd_name1_buf;
    cmd_name_arr[1].size     = 0;
    cmd_name_arr[1].capacity = sizeof(cmd_name1_buf);
    cmd_msg.name.data        = cmd_name_arr;
    cmd_msg.name.size        = 0;
    cmd_msg.name.capacity    = 2;

    cmd_msg.velocity.data     = cmd_vel_arr;
    cmd_msg.velocity.size     = 0;
    cmd_msg.velocity.capacity = 2;

    cmd_msg.position.data     = cmd_pos_arr;
    cmd_msg.position.size     = 0;
    cmd_msg.position.capacity = 2;

    cmd_msg.effort.data     = cmd_eff_arr;
    cmd_msg.effort.size     = 0;
    cmd_msg.effort.capacity = 2;

    cmd_msg.header.frame_id.data     = cmd_frame_id_buf;
    cmd_msg.header.frame_id.size     = 0;
    cmd_msg.header.frame_id.capacity = sizeof(cmd_frame_id_buf);

    emergency_stop_all();

    state = WAITING_AGENT;

    while (true) {
        watchdog_update();
        set_state_led(state);

        switch (state) {
            case WAITING_AGENT:
                emergency_stop_all();
                EXECUTE_EVERY_N_MS(AGENT_POLL_MS,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                        ? AGENT_AVAILABLE : WAITING_AGENT;
                );
                break;

            case AGENT_AVAILABLE:
                if (create_entities()) {
                    rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
                    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
                    state = AGENT_CONNECTED;
                    last_command_ms = now_ms();
                } else {
                    emergency_stop_all();
                    state = WAITING_AGENT;
                }
                break;

            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(AGENT_KEEPALIVE_MS,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                        ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                );
                EXECUTE_EVERY_N_MS(TIME_SYNC_PERIOD_MS,
                    rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
                );

                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&ctx.executor, RCL_MS_TO_NS(10));
                    update_control();
                    sleep_ms(1);
                }
                break;

            case AGENT_DISCONNECTED:
                emergency_stop_all();
                destroy_entities();
                state = WAITING_AGENT;
                break;

            default:
                emergency_stop_all();
                state = WAITING_AGENT;
                break;
        }
    }
}
