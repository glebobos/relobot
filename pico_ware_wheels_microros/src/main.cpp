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
#include "pid_controller.h"
#include "pico_uart_transports.h"

// --- Timing ---
#define PWM_WRAP                    12499U      // ~10 kHz at 125 MHz sys_clk
#define COMMAND_TIMEOUT_MS          2000U
#define WATCHDOG_TIMEOUT_MS         2000U
#define CONTROL_PERIOD_MS           10U         // 100 Hz
#define PUBLISH_PERIOD_MS           33U         // ~30 Hz

// --- Physical constants ---
#define COUNTS_PER_REVOLUTION       60.0f
#define TWO_PI                      (2.0f * (float)M_PI)
#define VELOCITY_EMA_ALPHA          0.25f

// --- Feedforward: pwm_fraction = K * |omega_rad_s| + C ---
// Right motor has different K in reverse (empirical, from code.py).
#define FF_K_LEFT_FWD               0.12281f
#define FF_C_LEFT_FWD               0.029637f
#define FF_K_LEFT_REV               0.12281f    // same as fwd (code.py uses same)
#define FF_C_LEFT_REV               0.029637f
#define FF_K_RIGHT_FWD              0.12681f
#define FF_C_RIGHT_FWD              0.021466f
#define FF_K_RIGHT_REV              0.11793f    // from code.py speed_to_pwm_right reverse
#define FF_C_RIGHT_REV              0.021466f

// --- PI tuning ---
// Output is a PWM *fraction* added to feedforward (range 0..1).
// With period-based velocity measurement the error is in true rad/s.
// Effective PWM gain = Kp_old(0.15) × FF_K(~0.123) ≈ 0.018 per rad/s error.
#define PI_KP                       0.018f
#define PI_KI                       0.005f
#define PI_INTEGRAL_LIMIT           0.15f

// --- Control limits ---
#define MAX_VEL_RAD_S               7.5f        // absolute clamp on incoming commands
#define SLEW_LIMIT_RAD_S_PER_CYCLE  0.5f        // max delta per CONTROL_PERIOD_MS step
#define MOTOR_DEADBAND_RAD_S        0.35f
// At 100 Hz with CPR=60, a motor at deadband speed (0.35 rad/s) produces one
// encoder tick every ~30 control cycles.  Set the stall threshold to 50 cycles
// (~500 ms) so normal low-speed gaps never false-trigger stall detection.
#define STALL_DETECT_CYCLES         50
#define STALL_GIVEUP_CYCLES         300         // ~3 s at 100 Hz: stop motor if truly stuck
#define STALL_BOOST_STEP            0.2f        // rad/s added per stall cycle
#define STALL_BOOST_MAX             3.5f        // rad/s maximum stall boost

// --- Pin assignments (XIAO RP2040 board labels -> GPIO) ---
#define PIN_LEFT_FWD_PWM            27          // D1, Slice 5 Ch B
#define PIN_LEFT_REV_PWM            26          // D0, Slice 5 Ch A
#define PIN_RIGHT_FWD_PWM           28          // D2, Slice 6 Ch A
#define PIN_RIGHT_REV_PWM           6           // D4, Slice 3 Ch A
#define PIN_ENCODER_LEFT            1           // D7, UART0_RX (stdio UART disabled)
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
    uint              fwd_pin;
    uint              rev_pin;
    uint              enc_pin;
    float             ff_k;
    float             ff_c;
    float             ff_k_rev;     // feedforward K for reverse direction
    float             ff_c_rev;     // feedforward C for reverse direction
    volatile uint32_t encoder_ticks;
    volatile uint32_t last_tick_us;    // µs timestamp of most recent tick (ISR-written)
    volatile uint32_t tick_period_us;  // µs between last two ticks  (ISR-written)
    uint32_t          last_enc_ticks;
    float             cmd_vel;          // rad/s from command callback
    float             target_vel;       // rad/s, slew-limited
    float             measured_vel;     // rad/s, EMA of period-based measurements
    int64_t           tick_accumulator; // signed tick count for drift-free position
    bool              last_dir_fwd;
    uint64_t          dir_change_at_ms; // wall-clock time of last direction change
    uint32_t          encoder_age;      // control cycles since last encoder tick
    float             stall_boost;      // extra rad/s to overcome stiction
    pid_controller_t  pid;
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
        pwm_set_gpio_level(m->fwd_pin, 0);
        pwm_set_gpio_level(m->rev_pin, 0);
        m->last_dir_fwd = fwd;
        return;
    }

    if (fwd != m->last_dir_fwd) {
        // Direction change: coast outputs and record timestamp — do NOT block.
        pwm_set_gpio_level(m->fwd_pin, 0);
        pwm_set_gpio_level(m->rev_pin, 0);
        m->dir_change_at_ms = now_ms();
        m->last_dir_fwd     = fwd;
        pid_controller_reset(&m->pid);
        return;  // coast this cycle; PWM applied once deadtime has elapsed
    }

    // Still within deadtime window — keep coasting.
    if ((now_ms() - m->dir_change_at_ms) < DIRECTION_DEADTIME_MS) {
        return;
    }

    if (fwd) {
        pwm_set_gpio_level(m->fwd_pin, level);
        pwm_set_gpio_level(m->rev_pin, 0);
    } else {
        pwm_set_gpio_level(m->fwd_pin, 0);
        pwm_set_gpio_level(m->rev_pin, level);
    }
}

static void stop_motor(motor_t *m)
{
    pwm_set_gpio_level(m->fwd_pin, 0);
    pwm_set_gpio_level(m->rev_pin, 0);
    pid_controller_reset(&m->pid);
    m->stall_boost  = 0.0f;
    m->encoder_age  = 0;     // reset so re-start doesn't immediately trigger stall
}

static void emergency_stop_all(void)
{
    stop_motor(&left_motor);
    stop_motor(&right_motor);
    left_motor.cmd_vel    = 0.0f;
    left_motor.target_vel = 0.0f;
    right_motor.cmd_vel    = 0.0f;
    right_motor.target_vel = 0.0f;
}

static void encoder_isr(uint gpio, uint32_t events)
{
    (void)events;
    uint32_t now_us = time_us_32();
    if (gpio == PIN_ENCODER_LEFT) {
        left_motor.tick_period_us = now_us - left_motor.last_tick_us;
        left_motor.last_tick_us   = now_us;
        left_motor.encoder_ticks++;
    } else if (gpio == PIN_ENCODER_RIGHT) {
        right_motor.tick_period_us = now_us - right_motor.last_tick_us;
        right_motor.last_tick_us   = now_us;
        right_motor.encoder_ticks++;
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
            new_left = clampf((float)msg->velocity.data[i], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
            got_left = true;
        } else if (name && strncmp(name, "right_wheel_joint", msg->name.data[i].size) == 0) {
            new_right = clampf((float)msg->velocity.data[i], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
            got_right = true;
        }
    }

    // Fall back to positional mapping if names are absent (e.g. testing with ros2 topic pub).
    if (!got_left && !got_right) {
        new_left  = clampf((float)msg->velocity.data[0], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
        new_right = clampf((float)msg->velocity.data[1], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
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
    // Guard against zero/invalid dt to prevent NaN/Inf in velocity computation.
    if (dt_s <= 0.0f || !isfinite(dt_s)) {
        return;
    }

    // --- Encoder delta (ISR-safe: read once) ---
    uint32_t ticks = m->encoder_ticks;
    uint32_t delta = ticks - m->last_enc_ticks;
    m->last_enc_ticks = ticks;

    // --- Velocity and position ---
    if (delta > 0) {
        // Period-based velocity: use the time between the last two encoder ticks.
        // At typical speeds (0.5–5 rad/s, CPR=60) the tick period (17–300 ms) is
        // much longer than the control loop dt (10 ms).  Using delta/dt_control
        // gives readings 10–30× too high, which causes the PI to suppress the
        // feedforward entirely.  tick_period_us is a single volatile uint32_t
        // written by the ISR — reads are atomic on ARM Cortex-M0+.
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
        // Accumulate ticks as int64 for drift-free position (avoids float rounding).
        m->tick_accumulator += m->last_dir_fwd ? (int64_t)delta : -(int64_t)delta;
        m->encoder_age  = 0;
        m->stall_boost  = 0.0f;
    } else {
        m->encoder_age++;
        // Decay measured_vel during stall so PID doesn't fight a phantom reading.
        if (m->encoder_age > STALL_DETECT_CYCLES) {
            m->measured_vel *= (1.0f - VELOCITY_EMA_ALPHA);
        }
    }

    // --- Slew limit ---
    float delta_cmd = m->cmd_vel - m->target_vel;
    delta_cmd = clampf(delta_cmd, -SLEW_LIMIT_RAD_S_PER_CYCLE, SLEW_LIMIT_RAD_S_PER_CYCLE);
    m->target_vel += delta_cmd;

    // --- Deadband ---
    if (fabsf(m->target_vel) < MOTOR_DEADBAND_RAD_S) {
        stop_motor(m);
        m->measured_vel *= (1.0f - VELOCITY_EMA_ALPHA);
        return;
    }

    // --- Stall detection and giveup ---
    if (m->encoder_age > STALL_DETECT_CYCLES && fabsf(m->cmd_vel) > MOTOR_DEADBAND_RAD_S) {
        // Give up entirely after the giveup threshold — stop applying current to
        // a mechanically blocked motor and let the operator re-command.
        if (m->encoder_age > STALL_GIVEUP_CYCLES) {
            m->cmd_vel    = 0.0f;
            m->target_vel = 0.0f;
            stop_motor(m);
            return;
        }
        m->stall_boost = fminf(m->stall_boost + STALL_BOOST_STEP, STALL_BOOST_MAX);
    }

    // --- Magnitude-based feedforward + PI ---
    bool  fwd          = (m->target_vel >= 0.0f);
    float abs_target   = fabsf(m->target_vel) + m->stall_boost;
    float abs_measured = fabsf(m->measured_vel);
    float ff_k         = fwd ? m->ff_k     : m->ff_k_rev;
    float ff_c         = fwd ? m->ff_c     : m->ff_c_rev;
    float ff_pwm       = clampf(ff_k * abs_target + ff_c, 0.0f, 1.0f);
    float pi_adj       = pid_controller_update(&m->pid, abs_target, abs_measured, dt_s);
    float out_pwm      = clampf(ff_pwm + pi_adj, 0.0f, 1.0f);

    set_motor_pwm(m, out_pwm, fwd);
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

    if (rclc_node_init_default(&ctx.node, "wheels_node", "", &ctx.support) != RCL_RET_OK)
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

static void init_motor_pwm(uint fwd_pin, uint rev_pin)
{
    gpio_set_function(fwd_pin, GPIO_FUNC_PWM);
    gpio_set_function(rev_pin, GPIO_FUNC_PWM);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    uint slice_fwd = pwm_gpio_to_slice_num(fwd_pin);
    uint slice_rev = pwm_gpio_to_slice_num(rev_pin);

    pwm_init(slice_fwd, &cfg, true);
    if (slice_rev != slice_fwd) {
        pwm_init(slice_rev, &cfg, true);
    }

    pwm_set_gpio_level(fwd_pin, 0);
    pwm_set_gpio_level(rev_pin, 0);
}

static void init_encoder_pin(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static void init_motor_struct(motor_t *m,
                               uint fwd_pin, uint rev_pin, uint enc_pin,
                               float ff_k, float ff_c,
                               float ff_k_rev, float ff_c_rev)
{
    memset(m, 0, sizeof(*m));
    m->fwd_pin      = fwd_pin;
    m->rev_pin      = rev_pin;
    m->enc_pin      = enc_pin;
    m->ff_k         = ff_k;
    m->ff_c         = ff_c;
    m->ff_k_rev     = ff_k_rev;
    m->ff_c_rev     = ff_c_rev;
    m->last_dir_fwd = true;
    pid_controller_init(
        &m->pid,
        PI_KP, PI_KI, 0.0f,
        -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT,
        -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
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
    init_motor_struct(&left_motor,
                      PIN_LEFT_FWD_PWM, PIN_LEFT_REV_PWM, PIN_ENCODER_LEFT,
                      FF_K_LEFT_FWD, FF_C_LEFT_FWD, FF_K_LEFT_REV, FF_C_LEFT_REV);
    init_motor_struct(&right_motor,
                      PIN_RIGHT_FWD_PWM, PIN_RIGHT_REV_PWM, PIN_ENCODER_RIGHT,
                      FF_K_RIGHT_FWD, FF_C_RIGHT_FWD, FF_K_RIGHT_REV, FF_C_RIGHT_REV);

    init_motor_pwm(PIN_LEFT_FWD_PWM,  PIN_LEFT_REV_PWM);
    init_motor_pwm(PIN_RIGHT_FWD_PWM, PIN_RIGHT_REV_PWM);

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
