#include <math.h>
#include <string.h>

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

#include "joint_state_codec.h"
#include "float32_codec.h"
#include "pid_controller.h"
#include "relobot/board.h"
#include "relobot/picoros_runtime.h"

/* Timing configuration */
#define PWM_WRAP                    9614U       /* ~13 kHz at 125 MHz sys_clk */
#define COMMAND_TIMEOUT_MS          2000U
#define WATCHDOG_TIMEOUT_MS         2000U
#define CONTROL_PERIOD_MS           10U         /* 100 Hz */
#define PUBLISH_PERIOD_MS           33U         /* ~30 Hz */
#define MAX_CONTROL_GAP_MS          100U
#define DIRECTION_DEADTIME_MS       2U

/* Physical constants */
#define COUNTS_PER_REVOLUTION       60.0f
#define TWO_PI                      (2.0f * (float)M_PI)
#define VELOCITY_EMA_ALPHA_TICK     0.4f

/* Feedforward: pwm_fraction = K * |omega_rad_s| + C */
#define FF_K_LEFT_FWD               0.135435f
#define FF_C_LEFT_FWD               0.041088f
#define FF_K_LEFT_REV               0.160302f
#define FF_C_LEFT_REV               0.026934f
#define FF_K_RIGHT_FWD              0.142810f
#define FF_C_RIGHT_FWD              0.046127f
#define FF_K_RIGHT_REV              0.153041f
#define FF_C_RIGHT_REV              0.026962f

/* PI tuning (PWM fraction addition, error in true rad/s) */
#define PI_KP                       0.10f
#define PI_KI                       0.30f
#define PI_INTEGRAL_LIMIT           0.15f
#define PI_UPDATE_INTERVAL_CYCLES   5U          /* update PI every 50ms (5 * 10ms) */

/* Control limits */
#define MAX_VEL_RAD_S               7.5f        /* absolute clamp on incoming commands */
#define SLEW_LIMIT_RAD_S_PER_CYCLE  0.5f        /* max delta per CONTROL_PERIOD_MS step */
#define MOTOR_DEADBAND_RAD_S        0.35f
#define STALL_DETECT_CYCLES         50U         /* ~500 ms at 100 Hz */
#define STALL_GIVEUP_CYCLES         300U        /* ~3 s at 100 Hz */
#define KICKSTART_PWM               0.60f       /* PWM fraction for stall kick-start pulse */
#define KICKSTART_DURATION_CYCLES   8U          /* 80 ms at 10 ms control period */

/* Pin assignments (XIAO RP2040 GPIOs) */
#define PIN_LEFT_PWM                27U         /* D1, Slice 5 Ch B */
#define PIN_LEFT_DIR                26U         /* D0 */
#define PIN_RIGHT_PWM               28U         /* D2, Slice 6 Ch A */
#define PIN_RIGHT_DIR               7U          /* D5 */
#define PIN_ENCODER_LEFT            6U          /* D4 */
#define PIN_ENCODER_RIGHT           3U          /* D10 */

/* FreeRTOS task stacks and priorities */
#define APP_TASK_STACK_WORDS        (configMINIMAL_STACK_SIZE * 8)
#define CONTROL_TASK_STACK_WORDS    (configMINIMAL_STACK_SIZE * 4)
#define APP_TASK_PRIORITY           (tskIDLE_PRIORITY + 2)
#define CONTROL_TASK_PRIORITY       (APP_TASK_PRIORITY + 1)

#define VEL_BUFF_SIZE 4

typedef struct {
    uint              pwm_pin;
    uint              dir_pin;
    uint              enc_pin;
    float             ff_k;
    float             ff_c;
    float             ff_k_rev;
    float             ff_c_rev;
    volatile uint32_t encoder_ticks;
    volatile uint32_t last_tick_us;
    volatile uint32_t tick_period_us;
    volatile uint32_t tick_buffer[VEL_BUFF_SIZE];
    volatile uint     buf_idx;
    uint32_t          last_enc_ticks;
    float             cmd_vel;
    float             target_vel;
    float             measured_vel;
    int64_t           tick_accumulator;
    bool              last_dir_fwd;
    uint32_t          dir_change_at_ms;
    uint32_t          encoder_age;
    uint32_t          kickstart_counter;
    uint32_t          pi_age;
    float             last_pi_adj;
    pid_controller_t  pid;
} motor_t;

static TaskHandle_t app_task_handle;
static TaskHandle_t control_task_handle;

static motor_t left_motor;
static motor_t right_motor;

static uint32_t last_command_ms = 0;
static uint32_t last_control_ms = 0;
static bool entities_ready = false;

static picoros_node_t node = {.name = "wheels_node"};
static volatile unsigned int diagnostic_stage = 1;

void picoros_diagnostic_stage(unsigned int stage)
{
    taskENTER_CRITICAL();
    diagnostic_stage = stage;
    taskEXIT_CRITICAL();
}

/* Preallocated publisher message */
static sensor_msgs__msg__JointState state_msg;
static rosidl_runtime_c__String     state_name_arr[2];
static double                       state_pos_arr[2];
static double                       state_vel_arr[2];
static double                       state_eff_arr[2];
static char                         state_name0[] = "left_wheel_joint";
static char                         state_name1[] = "right_wheel_joint";
static char                         state_frame_id_buf[16] = "";

/* Preallocated subscriber message */
static sensor_msgs__msg__JointState cmd_msg;
static rosidl_runtime_c__String     cmd_name_arr[2];
static double                       cmd_pos_arr[2];
static double                       cmd_vel_arr[2];
static double                       cmd_eff_arr[2];
static char                         cmd_name0_buf[32];
static char                         cmd_name1_buf[32];
static char                         cmd_frame_id_buf[32];

static picoros_publisher_t pub_states = {
    .topic = {
        .name = "robot_joint_states",
        .type = "sensor_msgs::msg::dds_::JointState",
    },
};

static void command_received(uint8_t *payload, size_t length);

static picoros_subscriber_t sub_commands = {
    .topic = {
        .name = "robot_joint_commands",
        .type = "sensor_msgs::msg::dds_::JointState",
    },
    .user_callback = command_received,
};

#if RELOBOT_WHEELS_BENCH
static picoros_publisher_t bench_publishers[] = {
    {.topic = {.name = "wheels/bench/target_left", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "wheels/bench/target_right", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "wheels/bench/heap_free_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "wheels/bench/heap_min_bytes", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "wheels/bench/app_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "wheels/bench/control_stack_free_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "wheels/bench/publish_errors", .type = "std_msgs::msg::dds_::Float32"}},
};
#endif

static inline uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void set_motor_pwm(motor_t *m, float pwm_fraction, bool fwd)
{
    uint16_t level = (uint16_t)(pid_clampf(pwm_fraction, 0.0f, 1.0f) * (float)PWM_WRAP);

    if (level == 0) {
#if !RELOBOT_WHEELS_BENCH
        pwm_set_gpio_level(m->pwm_pin, 0);
        gpio_put(m->dir_pin, 0);
#endif
        m->last_dir_fwd = fwd;
        return;
    }

    if (fwd != m->last_dir_fwd) {
#if !RELOBOT_WHEELS_BENCH
        pwm_set_gpio_level(m->pwm_pin, 0);
        gpio_put(m->dir_pin, 0);
#endif
        m->dir_change_at_ms = now_ms();
        m->last_dir_fwd     = fwd;
        pid_controller_reset(&m->pid);
        return;
    }

    if ((uint32_t)(now_ms() - m->dir_change_at_ms) < DIRECTION_DEADTIME_MS) {
        return;
    }

#if !RELOBOT_WHEELS_BENCH
    if (m->dir_pin == PIN_RIGHT_DIR) {
        gpio_put(m->dir_pin, fwd ? 1 : 0);
    } else {
        gpio_put(m->dir_pin, fwd ? 0 : 1);
    }
    pwm_set_gpio_level(m->pwm_pin, level);
#endif
}

static void stop_motor(motor_t *m)
{
#if !RELOBOT_WHEELS_BENCH
    pwm_set_gpio_level(m->pwm_pin, 0);
    gpio_put(m->dir_pin, 0);
#endif
    pid_controller_reset(&m->pid);
    m->kickstart_counter = 0;
    m->last_pi_adj       = 0.0f;
    m->encoder_age       = 0;
}

static void emergency_stop_all(void)
{
    stop_motor(&left_motor);
    stop_motor(&right_motor);
    left_motor.cmd_vel    = 0.0f;
    left_motor.target_vel = 0.0f;
    right_motor.cmd_vel   = 0.0f;
    right_motor.target_vel = 0.0f;
}

void relobot_fault(void)
{
    (void)save_and_disable_interrupts();
    emergency_stop_all();
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
        emergency_stop_all();
        reset_usb_boot(0, 0);
    }
}

#if !RELOBOT_WHEELS_BENCH
static void encoder_isr(uint gpio, uint32_t events)
{
    (void)events;
    uint32_t now_us = time_us_32();
    if (gpio == PIN_ENCODER_LEFT) {
        uint32_t diff = now_us - left_motor.last_tick_us;
        if (diff > 10000U) {  /* 10 ms debounce */
            left_motor.last_tick_us = now_us;
            left_motor.encoder_ticks++;
            left_motor.tick_buffer[left_motor.buf_idx] = diff;
            left_motor.buf_idx = (left_motor.buf_idx + 1) % VEL_BUFF_SIZE;
            uint32_t sum = 0;
            for (int i = 0; i < VEL_BUFF_SIZE; i++) {
                sum += left_motor.tick_buffer[i];
            }
            left_motor.tick_period_us = sum / VEL_BUFF_SIZE;
        }
    } else if (gpio == PIN_ENCODER_RIGHT) {
        uint32_t diff = now_us - right_motor.last_tick_us;
        if (diff > 10000U) {  /* 10 ms debounce */
            right_motor.last_tick_us = now_us;
            right_motor.encoder_ticks++;
            right_motor.tick_buffer[right_motor.buf_idx] = diff;
            right_motor.buf_idx = (right_motor.buf_idx + 1) % VEL_BUFF_SIZE;
            uint32_t sum = 0;
            for (int i = 0; i < VEL_BUFF_SIZE; i++) {
                sum += right_motor.tick_buffer[i];
            }
            right_motor.tick_period_us = sum / VEL_BUFF_SIZE;
        }
    }
}
#endif

static bool extract_joint_velocities(const sensor_msgs__msg__JointState *msg, float *left_vel, float *right_vel)
{
    if (msg == NULL || msg->velocity.size < 2 || left_vel == NULL || right_vel == NULL) {
        return false;
    }

    bool got_left = false;
    bool got_right = false;
    float l = 0.0f;
    float r = 0.0f;

    for (size_t i = 0; i < msg->name.size && i < msg->velocity.size; i++) {
        const char *name = msg->name.data[i].data;
        size_t name_len = msg->name.data[i].size;
        if (name != NULL) {
            if (strncmp(name, "left_wheel_joint", name_len) == 0 && name_len == strlen("left_wheel_joint")) {
                if (!isfinite(msg->velocity.data[i])) return false;
                l = pid_clampf((float)msg->velocity.data[i], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
                got_left = true;
            } else if (strncmp(name, "right_wheel_joint", name_len) == 0 && name_len == strlen("right_wheel_joint")) {
                if (!isfinite(msg->velocity.data[i])) return false;
                r = pid_clampf((float)msg->velocity.data[i], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
                got_right = true;
            }
        }
    }

    /* Positional fallback if names were not specified or didn't match */
    if (!got_left && !got_right) {
        if (!isfinite(msg->velocity.data[0]) || !isfinite(msg->velocity.data[1])) return false;
        l = pid_clampf((float)msg->velocity.data[0], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
        r = pid_clampf((float)msg->velocity.data[1], -MAX_VEL_RAD_S, MAX_VEL_RAD_S);
        got_left = true;
        got_right = true;
    }

    if (got_left && got_right) {
        *left_vel = l;
        *right_vel = r;
        return true;
    }
    return false;
}

static void command_received(uint8_t *payload, size_t length)
{
    joint_state_msg_init_cmd(&cmd_msg, cmd_name_arr,
                             cmd_name0_buf, sizeof(cmd_name0_buf),
                             cmd_name1_buf, sizeof(cmd_name1_buf),
                             cmd_pos_arr, cmd_vel_arr, cmd_eff_arr,
                             cmd_frame_id_buf, sizeof(cmd_frame_id_buf));

    if (joint_state_codec_decode(payload, length, &cmd_msg)) {
        float new_left = 0.0f;
        float new_right = 0.0f;
        if (extract_joint_velocities(&cmd_msg, &new_left, &new_right)) {
            taskENTER_CRITICAL();
            left_motor.cmd_vel  = new_left;
            right_motor.cmd_vel = new_right;
            last_command_ms     = now_ms();
            taskEXIT_CRITICAL();
        }
    }
}

static void update_motor(motor_t *m, float dt_s)
{
    if (dt_s <= 0.0f || !isfinite(dt_s)) {
        return;
    }

    /* Encoder delta (ISR-safe atomic read) */
    uint32_t ticks = m->encoder_ticks;
    uint32_t delta = ticks - m->last_enc_ticks;
    m->last_enc_ticks = ticks;

    float direction = m->last_dir_fwd ? 1.0f : -1.0f;

    /* Velocity and position estimation */
    if (delta > 0) {
        uint32_t period_us = m->tick_period_us;
        if (period_us > 500U && period_us < 5000000U) {
            float raw_vel = direction * TWO_PI
                            / (COUNTS_PER_REVOLUTION * ((float)period_us * 1e-6f));
            if (isfinite(raw_vel)) {
                m->measured_vel = VELOCITY_EMA_ALPHA_TICK * raw_vel
                                + (1.0f - VELOCITY_EMA_ALPHA_TICK) * m->measured_vel;
            }
        }
        m->tick_accumulator += m->last_dir_fwd ? (int64_t)delta : -(int64_t)delta;
        m->encoder_age       = 0;
        m->kickstart_counter = 0;
    } else {
        m->encoder_age++;
        uint32_t age_us    = time_us_32() - m->last_tick_us;
        uint32_t period_us = m->tick_period_us;
        if (age_us > period_us && age_us > 500U && age_us < 5000000U) {
            float raw_vel = direction * TWO_PI
                            / (COUNTS_PER_REVOLUTION * ((float)age_us * 1e-6f));
            if (isfinite(raw_vel)) {
                m->measured_vel = raw_vel;
            }
        }
    }

    /* Slew limit */
    float delta_cmd = m->cmd_vel - m->target_vel;
    delta_cmd = pid_clampf(delta_cmd, -SLEW_LIMIT_RAD_S_PER_CYCLE, SLEW_LIMIT_RAD_S_PER_CYCLE);
    m->target_vel += delta_cmd;

    /* Deadband */
    if (fabsf(m->target_vel) < MOTOR_DEADBAND_RAD_S) {
        stop_motor(m);
        uint32_t age_us = time_us_32() - m->last_tick_us;
        if (age_us > m->tick_period_us && age_us > 500U && age_us < 5000000U) {
            m->measured_vel = direction * TWO_PI
                              / (COUNTS_PER_REVOLUTION * ((float)age_us * 1e-6f));
        }
        return;
    }

    /* Stall detection & kick-start pulse */
    if (m->encoder_age > STALL_DETECT_CYCLES && fabsf(m->cmd_vel) > MOTOR_DEADBAND_RAD_S) {
        if (m->encoder_age > STALL_GIVEUP_CYCLES) {
            m->cmd_vel    = 0.0f;
            m->target_vel = 0.0f;
            stop_motor(m);
            return;
        }
        if (m->kickstart_counter == 0) {
            m->kickstart_counter = KICKSTART_DURATION_CYCLES;
            pid_controller_reset(&m->pid);
            m->last_pi_adj = 0.0f;
        }
    }
    if (m->kickstart_counter > 0) {
        bool fwd = (m->target_vel >= 0.0f);
        set_motor_pwm(m, KICKSTART_PWM, fwd);
        m->kickstart_counter--;
        return;
    }

    /* Feedforward calculation */
    bool  fwd    = (m->target_vel >= 0.0f);
    float ff_k   = fwd ? m->ff_k : m->ff_k_rev;
    float ff_c   = fwd ? m->ff_c : m->ff_c_rev;
    float ff_pwm = pid_clampf(ff_k * fabsf(m->target_vel) + ff_c, 0.0f, 1.0f);

    /* Throttled PI controller */
    m->pi_age++;
    bool run_pi = (delta > 0) || (m->pi_age >= PI_UPDATE_INTERVAL_CYCLES);
    if (run_pi) {
        float pi_dt = (float)m->pi_age * ((float)CONTROL_PERIOD_MS / 1000.0f);
        m->last_pi_adj = pid_controller_update(&m->pid, m->target_vel, m->measured_vel, pi_dt);
        m->pi_age = 0;
    }

    float out_pwm = pid_clampf(ff_pwm + m->last_pi_adj, 0.0f, 1.0f);
    set_motor_pwm(m, out_pwm, fwd);
}

static void update_control(void)
{
    uint32_t now = now_ms();

    if (last_control_ms == 0) {
        last_control_ms = now;
        last_command_ms = now;
        return;
    }

    /* Max control gap check */
    if ((uint32_t)(now - last_control_ms) > MAX_CONTROL_GAP_MS) {
        emergency_stop_all();
        last_control_ms = now;
        return;
    }

    /* Command timeout */
    if ((uint32_t)(now - last_command_ms) > COMMAND_TIMEOUT_MS) {
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

static void control_task(void *argument)
{
    (void)argument;
    TickType_t previous_wake = xTaskGetTickCount();

    for (;;) {
        update_control();

        taskENTER_CRITICAL();
        bool ready = entities_ready;
        unsigned int stage = diagnostic_stage;
        bool active = ready && (fabsf(left_motor.cmd_vel) >= MOTOR_DEADBAND_RAD_S ||
                                fabsf(right_motor.cmd_vel) >= MOTOR_DEADBAND_RAD_S);
        taskEXIT_CRITICAL();

        uint32_t now = now_ms();
        uint8_t r = 0, g = 0, b = 0;
        if ((now % 1000) < 100) {
            /* Heartbeat: brief 100 ms off phase every second */
            r = 0; g = 0; b = 0;
        } else if (stage == 0) {
            b = 32; /* Solid Blue: codec_init failed */
        } else if (stage >= 1 && stage <= 5) {
            r = 32; /* Red: waiting/attempting Zenoh session init */
        } else if (ready && stage >= 6 && stage <= 10) {
            if (active) {
                g = 32; /* Green: wheels active/moving */
            } else {
                r = 24; g = 12; /* Dim amber: ready/idle */
            }
        } else if (stage >= 6 && stage <= 9) {
            r = 24; g = 20; /* Amber: Zenoh session open, declaring entities */
        } else {
            r = 32; b = 16; /* Magenta: unexpected stage */
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
    emergency_stop_all();
    taskEXIT_CRITICAL();
    picoros_diagnostic_stage(stage);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#if RELOBOT_WHEELS_BENCH
static bool publish_float(picoros_publisher_t *publisher, float value)
{
    uint8_t payload[FLOAT32_CDR_SIZE];
    return float32_codec_encode(value, payload) &&
           picoros_publish(publisher, payload, sizeof(payload)) == PICOROS_OK;
}
#endif

static void app_task(void *argument)
{
    (void)argument;

    if (!joint_state_codec_init()) {
        startup_failed(0);
    }
#if RELOBOT_WHEELS_BENCH
    if (!float32_codec_init()) {
        startup_failed(0);
    }
#endif

    sub_commands.topic.rihs_hash = joint_state_codec_hash();
    pub_states.topic.rihs_hash = joint_state_codec_hash();

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
        z_publisher_options_default(&pub_states.opts);
        if (picoros_publisher_declare(&node, &pub_states) != PICOROS_OK) {
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

#if RELOBOT_WHEELS_BENCH
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
            (void)picoros_publisher_drop(&pub_states);
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
#endif

        picoros_diagnostic_stage(9);
        if (picoros_subscriber_declare(&node, &sub_commands) != PICOROS_OK) {
#if RELOBOT_WHEELS_BENCH
            for (size_t index = 0; index < sizeof(bench_publishers) / sizeof(bench_publishers[0]); ++index) {
                (void)picoros_publisher_drop(&bench_publishers[index]);
            }
#endif
            (void)picoros_publisher_drop(&pub_states);
            (void)picoros_node_drop(&node);
            picoros_interface_close();
            picoros_diagnostic_stage(5);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        break;
    }

    taskENTER_CRITICAL();
    entities_ready = true;
    diagnostic_stage = 10;
    taskEXIT_CRITICAL();

#if RELOBOT_WHEELS_BENCH
    uint32_t publish_errors = 0;
    uint32_t last_diagnostics_ms = now_ms();
#endif

    for (;;) {
        taskENTER_CRITICAL();
        int64_t left_ticks = left_motor.tick_accumulator;
        int64_t right_ticks = right_motor.tick_accumulator;
        float left_vel = left_motor.measured_vel;
        float right_vel = right_motor.measured_vel;
#if RELOBOT_WHEELS_BENCH
        float target_left = left_motor.cmd_vel;
        float target_right = right_motor.cmd_vel;
#endif
        taskEXIT_CRITICAL();

        state_pos_arr[0] = (double)left_ticks * ((double)TWO_PI / (double)COUNTS_PER_REVOLUTION);
        state_pos_arr[1] = (double)right_ticks * ((double)TWO_PI / (double)COUNTS_PER_REVOLUTION);
        state_vel_arr[0] = (double)left_vel;
        state_vel_arr[1] = (double)right_vel;
        state_eff_arr[0] = 0.0;
        state_eff_arr[1] = 0.0;

        uint64_t now_us = to_us_since_boot(get_absolute_time());
        state_msg.header.stamp.sec = (int32_t)(now_us / 1000000ULL);
        state_msg.header.stamp.nanosec = (uint32_t)((now_us % 1000000ULL) * 1000ULL);

        uint8_t payload[JOINT_STATE_CDR_MAX_SIZE];
        size_t actual_length = 0;
        if (joint_state_codec_encode(&state_msg, payload, sizeof(payload), &actual_length)) {
            if (picoros_publish(&pub_states, payload, actual_length) != PICOROS_OK) {
#if RELOBOT_WHEELS_BENCH
                publish_errors++;
#endif
            }
        } else {
#if RELOBOT_WHEELS_BENCH
            publish_errors++;
#endif
        }

#if RELOBOT_WHEELS_BENCH
        uint32_t now = now_ms();
        if ((uint32_t)(now - last_diagnostics_ms) >= 1000U) {
            float values[] = {
                target_left,
                target_right,
                (float)xPortGetFreeHeapSize(),
                (float)xPortGetMinimumEverFreeHeapSize(),
                (float)uxTaskGetStackHighWaterMark(app_task_handle),
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

        vTaskDelay(pdMS_TO_TICKS(PUBLISH_PERIOD_MS));
    }
}

static void init_motor_struct(motor_t *m,
                              uint pwm_pin, uint dir_pin, uint enc_pin,
                              float ff_k, float ff_c,
                              float ff_k_rev, float ff_c_rev)
{
    memset(m, 0, sizeof(*m));
    m->pwm_pin      = pwm_pin;
    m->dir_pin      = dir_pin;
    m->enc_pin      = enc_pin;
    m->ff_k         = ff_k;
    m->ff_c         = ff_c;
    m->ff_k_rev     = ff_k_rev;
    m->ff_c_rev     = ff_c_rev;
    m->last_dir_fwd = true;
    m->last_tick_us = time_us_32();
    for (int i = 0; i < VEL_BUFF_SIZE; i++) {
        m->tick_buffer[i] = 2000000U;
    }
    pid_controller_init(
        &m->pid,
        PI_KP, PI_KI, 0.0f,
        -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT,
        -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
}

static void init_hardware_io(void)
{
#if !RELOBOT_WHEELS_BENCH
    /* Motor PWM & DIR pins */
    gpio_set_function(PIN_LEFT_PWM, GPIO_FUNC_PWM);
    uint slice_left = pwm_gpio_to_slice_num(PIN_LEFT_PWM);
    pwm_config cfg_l = pwm_get_default_config();
    pwm_config_set_wrap(&cfg_l, PWM_WRAP);
    pwm_init(slice_left, &cfg_l, true);
    pwm_set_gpio_level(PIN_LEFT_PWM, 0);

    gpio_init(PIN_LEFT_DIR);
    gpio_set_dir(PIN_LEFT_DIR, GPIO_OUT);
    gpio_put(PIN_LEFT_DIR, 0);

    gpio_set_function(PIN_RIGHT_PWM, GPIO_FUNC_PWM);
    uint slice_right = pwm_gpio_to_slice_num(PIN_RIGHT_PWM);
    pwm_config cfg_r = pwm_get_default_config();
    pwm_config_set_wrap(&cfg_r, PWM_WRAP);
    pwm_init(slice_right, &cfg_r, true);
    pwm_set_gpio_level(PIN_RIGHT_PWM, 0);

    gpio_init(PIN_RIGHT_DIR);
    gpio_set_dir(PIN_RIGHT_DIR, GPIO_OUT);
    gpio_put(PIN_RIGHT_DIR, 0);

    /* Encoder pins with pull-up and rising edge IRQ */
    gpio_init(PIN_ENCODER_LEFT);
    gpio_set_dir(PIN_ENCODER_LEFT, GPIO_IN);
    gpio_pull_up(PIN_ENCODER_LEFT);

    gpio_init(PIN_ENCODER_RIGHT);
    gpio_set_dir(PIN_ENCODER_RIGHT, GPIO_IN);
    gpio_pull_up(PIN_ENCODER_RIGHT);

    gpio_set_irq_enabled_with_callback(PIN_ENCODER_LEFT, GPIO_IRQ_EDGE_RISE, true, &encoder_isr);
    gpio_set_irq_enabled(PIN_ENCODER_RIGHT, GPIO_IRQ_EDGE_RISE, true);
#endif
}

int main(void)
{
    init_motor_struct(&left_motor,
                      PIN_LEFT_PWM, PIN_LEFT_DIR, PIN_ENCODER_LEFT,
                      FF_K_LEFT_FWD, FF_C_LEFT_FWD, FF_K_LEFT_REV, FF_C_LEFT_REV);
    init_motor_struct(&right_motor,
                      PIN_RIGHT_PWM, PIN_RIGHT_DIR, PIN_ENCODER_RIGHT,
                      FF_K_RIGHT_FWD, FF_C_RIGHT_FWD, FF_K_RIGHT_REV, FF_C_RIGHT_REV);

    init_hardware_io();
    emergency_stop_all();

    /* Pre-initialize publisher message buffers */
    joint_state_msg_init_state(&state_msg, state_name_arr,
                               state_name0, state_name1,
                               state_pos_arr, state_vel_arr, state_eff_arr,
                               state_frame_id_buf, sizeof(state_frame_id_buf));

    watchdog_enable(WATCHDOG_TIMEOUT_MS, false);
    relobot_board_init();

    if (xTaskCreate(control_task, "wheels_control", CONTROL_TASK_STACK_WORDS,
                    NULL, CONTROL_TASK_PRIORITY, &control_task_handle) != pdPASS ||
        xTaskCreate(app_task, "picoros_app", APP_TASK_STACK_WORDS,
                    NULL, APP_TASK_PRIORITY, &app_task_handle) != pdPASS) {
        emergency_stop_all();
        for (;;) { tight_loop_contents(); }
    }

    vTaskStartScheduler();

    emergency_stop_all();
    for (;;) { tight_loop_contents(); }
}
