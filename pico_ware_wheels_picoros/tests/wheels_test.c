#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define main firmware_main
#include "../src/main.c"
#undef main

/* Mock/Stub state */
static bool reboot_requested = false;
static bool reset_usb_boot_called = false;
static jmp_buf fault_return;
static uint32_t test_time_ms = 0;
static uint32_t test_time_us = 0;
static bool gpio_levels[32];
static uint16_t pwm_levels[32];

/* Platform stubs implementation */
absolute_time_t get_absolute_time(void) { return (uint64_t)test_time_ms; }
uint32_t to_ms_since_boot(absolute_time_t time) { return (uint32_t)time; }
uint64_t to_us_since_boot(absolute_time_t time) { return (uint64_t)time * 1000ULL; }
uint32_t time_us_32(void) { return test_time_us; }
void sleep_ms(uint32_t ms) { test_time_ms += ms; test_time_us += ms * 1000U; }
uint32_t save_and_disable_interrupts(void) { return 0; }
void tight_loop_contents(void) { assert(false); }

void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin, bool output) { (void)pin; (void)output; }
void gpio_put(uint pin, bool value) { if (pin < 32) gpio_levels[pin] = value; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
void gpio_pull_up(uint pin) { (void)pin; }
void gpio_set_irq_enabled(uint pin, uint32_t events, bool enabled) { (void)pin; (void)events; (void)enabled; }
void gpio_set_irq_enabled_with_callback(uint pin, uint32_t events, bool enabled,
                                       void (*callback)(uint, uint32_t))
{
    (void)pin; (void)events; (void)enabled; (void)callback;
}

void pwm_set_gpio_level(uint pin, uint16_t level) { if (pin < 32) pwm_levels[pin] = level; }
uint pwm_gpio_to_slice_num(uint pin) { return pin / 2; }
pwm_config pwm_get_default_config(void) { pwm_config c = {0}; return c; }
void pwm_config_set_wrap(pwm_config *config, uint16_t wrap) { (void)config; (void)wrap; }
void pwm_init(uint slice, const pwm_config *config, bool enabled) { (void)slice; (void)config; (void)enabled; }

void watchdog_reboot(uint32_t pc, uint32_t sp, uint32_t delay_ms)
{
    (void)pc; (void)sp; (void)delay_ms;
    reboot_requested = true;
    longjmp(fault_return, 1);
}

void reset_usb_boot(uint32_t gpio_mask, uint32_t disable_mask)
{
    (void)gpio_mask; (void)disable_mask;
    reset_usb_boot_called = true;
}

void watchdog_enable(uint32_t delay_ms, bool pause_on_debug) { (void)delay_ms; (void)pause_on_debug; }
void watchdog_update(void) {}
TickType_t xTaskGetTickCount(void) { return (TickType_t)test_time_ms; }
void vTaskDelay(TickType_t ticks) { test_time_ms += ticks; test_time_us += ticks * 1000U; }
void vTaskDelayUntil(TickType_t *previous, TickType_t ticks) { (void)previous; (void)ticks; }
int xTaskCreate(void (*task)(void *), const char *name, uint32_t depth,
                void *argument, unsigned int priority, TaskHandle_t *handle)
{
    (void)task; (void)name; (void)depth; (void)argument; (void)priority; (void)handle;
    return pdPASS;
}
void vTaskStartScheduler(void) {}
size_t xPortGetFreeHeapSize(void) { return 65536; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return 32768; }
unsigned int uxTaskGetStackHighWaterMark(TaskHandle_t task) { (void)task; return 256; }
picoros_res_t picoros_node_init(picoros_node_t *node) { (void)node; return PICOROS_OK; }
picoros_res_t picoros_publisher_declare(picoros_node_t *node, picoros_publisher_t *publisher)
{
    (void)node; (void)publisher; return PICOROS_OK;
}
picoros_res_t picoros_subscriber_declare(picoros_node_t *node, picoros_subscriber_t *subscriber)
{
    (void)node; (void)subscriber; return PICOROS_OK;
}
picoros_res_t picoros_publish(picoros_publisher_t *publisher, const uint8_t *payload, size_t length)
{
    (void)publisher; (void)payload; (void)length; return PICOROS_OK;
}
void z_publisher_options_default(int *options) { (void)options; }
picoros_res_t relobot_picoros_init(void) { return PICOROS_OK; }
void relobot_board_init(void) {}
void relobot_board_led_set(uint8_t red, uint8_t green, uint8_t blue) { (void)red; (void)green; (void)blue; }

/* Stub codec implementations for unit test harness */
bool joint_state_codec_init(void) { return true; }
const char *joint_state_codec_hash(void) { return "1234567890abcdef"; }
bool joint_state_codec_encode(const sensor_msgs__msg__JointState *msg,
                              uint8_t *payload, size_t max_len, size_t *actual_length)
{
    (void)msg; (void)payload; (void)max_len;
    if (actual_length) *actual_length = 64;
    return true;
}
bool joint_state_codec_decode(const uint8_t *payload, size_t length,
                              sensor_msgs__msg__JointState *msg)
{
    (void)payload; (void)length; (void)msg;
    return true;
}

void joint_state_msg_init_state(sensor_msgs__msg__JointState *msg,
                                rosidl_runtime_c__String *name_arr,
                                char *name0,
                                char *name1,
                                double *pos_arr,
                                double *vel_arr,
                                double *eff_arr,
                                char *frame_id_buf,
                                size_t frame_id_cap)
{
    if (!msg) return;
    memset(msg, 0, sizeof(*msg));

    if (name_arr && name0 && name1) {
        name_arr[0].data = name0;
        name_arr[0].size = strlen(name0);
        name_arr[0].capacity = strlen(name0) + 1;

        name_arr[1].data = name1;
        name_arr[1].size = strlen(name1);
        name_arr[1].capacity = strlen(name1) + 1;

        msg->name.data = name_arr;
        msg->name.size = 2;
        msg->name.capacity = 2;
    }

    if (pos_arr) {
        msg->position.data = pos_arr;
        msg->position.size = 2;
        msg->position.capacity = 2;
    }

    if (vel_arr) {
        msg->velocity.data = vel_arr;
        msg->velocity.size = 2;
        msg->velocity.capacity = 2;
    }

    if (eff_arr) {
        msg->effort.data = eff_arr;
        msg->effort.size = 2;
        msg->effort.capacity = 2;
    }

    if (frame_id_buf) {
        msg->header.frame_id.data = frame_id_buf;
        msg->header.frame_id.size = strlen(frame_id_buf);
        msg->header.frame_id.capacity = frame_id_cap;
    }
}

void joint_state_msg_init_cmd(sensor_msgs__msg__JointState *msg,
                              rosidl_runtime_c__String *name_arr,
                              char *name0_buf,
                              size_t name0_cap,
                              char *name1_buf,
                              size_t name1_cap,
                              double *pos_arr,
                              double *vel_arr,
                              double *eff_arr,
                              char *frame_id_buf,
                              size_t frame_id_cap)
{
    if (!msg) return;
    memset(msg, 0, sizeof(*msg));

    if (name_arr && name0_buf && name1_buf) {
        name0_buf[0] = '\0';
        name1_buf[0] = '\0';
        name_arr[0].data = name0_buf;
        name_arr[0].size = 0;
        name_arr[0].capacity = name0_cap;

        name_arr[1].data = name1_buf;
        name_arr[1].size = 0;
        name_arr[1].capacity = name1_cap;

        msg->name.data = name_arr;
        msg->name.size = 0;
        msg->name.capacity = 2;
    }

    if (pos_arr) {
        msg->position.data = pos_arr;
        msg->position.size = 0;
        msg->position.capacity = 2;
    }

    if (vel_arr) {
        msg->velocity.data = vel_arr;
        msg->velocity.size = 0;
        msg->velocity.capacity = 2;
    }

    if (eff_arr) {
        msg->effort.data = eff_arr;
        msg->effort.size = 0;
        msg->effort.capacity = 2;
    }

    if (frame_id_buf) {
        frame_id_buf[0] = '\0';
        msg->header.frame_id.data = frame_id_buf;
        msg->header.frame_id.size = 0;
        msg->header.frame_id.capacity = frame_id_cap;
    }
}

bool float32_codec_init(void) { return true; }
const char *float32_codec_hash(void) { return "abcdef1234567890"; }
bool float32_codec_encode(float value, uint8_t payload[FLOAT32_CDR_SIZE])
{
    (void)value; (void)payload; return true;
}
bool float32_codec_decode(uint8_t *payload, size_t length, float *value)
{
    (void)payload; (void)length; if (value) *value = 0.0f; return true;
}

/* ----------------- TEST CASES ----------------- */

static void reset_test_state(void)
{
    memset(gpio_levels, 0, sizeof(gpio_levels));
    memset(pwm_levels, 0, sizeof(pwm_levels));
    test_time_ms = 1000;
    test_time_us = 1000000;
    last_command_ms = 1000;
    last_control_ms = 1000;
    reboot_requested = false;
    reset_usb_boot_called = false;
    entities_ready = true;

    init_motor_struct(&left_motor,
                      PIN_LEFT_PWM, PIN_LEFT_DIR, PIN_ENCODER_LEFT,
                      FF_K_LEFT_FWD, FF_C_LEFT_FWD, FF_K_LEFT_REV, FF_C_LEFT_REV);
    init_motor_struct(&right_motor,
                      PIN_RIGHT_PWM, PIN_RIGHT_DIR, PIN_ENCODER_RIGHT,
                      FF_K_RIGHT_FWD, FF_C_RIGHT_FWD, FF_K_RIGHT_REV, FF_C_RIGHT_REV);
}

static void test_defaults(void)
{
    reset_test_state();

    sensor_msgs__msg__JointState test_cmd;
    rosidl_runtime_c__String names[2];
    char n0[32], n1[32], fid[32];
    double p[2], v[2], e[2];

    joint_state_msg_init_cmd(&test_cmd, names, n0, sizeof(n0), n1, sizeof(n1),
                             p, v, e, fid, sizeof(fid));

    assert(test_cmd.name.capacity == 2);
    assert(test_cmd.name.size == 0);
    assert(test_cmd.velocity.capacity == 2);
    assert(test_cmd.velocity.size == 0);
    assert(test_cmd.position.capacity == 2);
    assert(test_cmd.effort.capacity == 2);

    sensor_msgs__msg__JointState test_state;
    char s0[] = "left_wheel_joint";
    char s1[] = "right_wheel_joint";
    joint_state_msg_init_state(&test_state, names, s0, s1, p, v, e, fid, sizeof(fid));

    assert(test_state.name.size == 2);
    assert(test_state.name.capacity == 2);
    assert(strcmp(test_state.name.data[0].data, "left_wheel_joint") == 0);
    assert(strcmp(test_state.name.data[1].data, "right_wheel_joint") == 0);
    assert(test_state.velocity.size == 2);
    assert(test_state.position.size == 2);
    assert(test_state.effort.size == 2);
}

static void test_commands(void)
{
    reset_test_state();

    sensor_msgs__msg__JointState msg;
    rosidl_runtime_c__String names[2];
    char n0[] = "left_wheel_joint";
    char n1[] = "right_wheel_joint";
    double vel[2] = {2.5, -1.8};
    float left = 0.0f, right = 0.0f;

    names[0].data = n0; names[0].size = strlen(n0); names[0].capacity = sizeof(n0);
    names[1].data = n1; names[1].size = strlen(n1); names[1].capacity = sizeof(n1);

    msg.name.data = names; msg.name.size = 2; msg.name.capacity = 2;
    msg.velocity.data = vel; msg.velocity.size = 2; msg.velocity.capacity = 2;

    /* 1. In-order named match */
    assert(extract_joint_velocities(&msg, &left, &right));
    assert(fabsf(left - 2.5f) < 1e-4f);
    assert(fabsf(right - (-1.8f)) < 1e-4f);

    /* 2. Reversed order named match */
    names[0].data = n1; names[0].size = strlen(n1);
    names[1].data = n0; names[1].size = strlen(n0);
    vel[0] = 3.2; vel[1] = -4.1;
    assert(extract_joint_velocities(&msg, &left, &right));
    assert(fabsf(left - (-4.1f)) < 1e-4f);
    assert(fabsf(right - 3.2f) < 1e-4f);

    /* 3. Positional fallback when names are absent */
    msg.name.size = 0;
    vel[0] = 1.0; vel[1] = -2.0;
    assert(extract_joint_velocities(&msg, &left, &right));
    assert(fabsf(left - 1.0f) < 1e-4f);
    assert(fabsf(right - (-2.0f)) < 1e-4f);

    /* 4. Non-finite rejection */
    vel[0] = (double)NAN; vel[1] = 1.0;
    assert(!extract_joint_velocities(&msg, &left, &right));

    vel[0] = 1.0; vel[1] = (double)INFINITY;
    assert(!extract_joint_velocities(&msg, &left, &right));

    /* 5. Clamping to MAX_VEL_RAD_S */
    vel[0] = 15.0; vel[1] = -20.0;
    assert(extract_joint_velocities(&msg, &left, &right));
    assert(fabsf(left - MAX_VEL_RAD_S) < 1e-4f);
    assert(fabsf(right - (-MAX_VEL_RAD_S)) < 1e-4f);
}

static void test_control(void)
{
    reset_test_state();

    /* Set target speeds */
    left_motor.cmd_vel = 3.0f;
    right_motor.cmd_vel = -3.0f;

    /* 1. Slew rate limiting check */
    test_time_ms = 1010;
    update_control();
    assert(left_motor.target_vel > 0.0f);
    assert(left_motor.target_vel <= SLEW_LIMIT_RAD_S_PER_CYCLE);
    assert(right_motor.target_vel < 0.0f);
    assert(right_motor.target_vel >= -SLEW_LIMIT_RAD_S_PER_CYCLE);

    /* Ramp up over multiple cycles */
    for (int i = 0; i < 20; i++) {
        test_time_ms += 10;
        update_control();
    }
    assert(fabsf(left_motor.target_vel - 3.0f) < 1e-4f);
    assert(fabsf(right_motor.target_vel - (-3.0f)) < 1e-4f);

    /* 2. Deadband check: when commanded speed is in deadband, motor stops */
    left_motor.cmd_vel = 0.2f; /* Below MOTOR_DEADBAND_RAD_S = 0.35 */
    right_motor.cmd_vel = 0.0f;
    for (int i = 0; i < 20; i++) {
        test_time_ms += 10;
        update_control();
    }
    assert(left_motor.target_vel < MOTOR_DEADBAND_RAD_S);
    assert(left_motor.kickstart_counter == 0);
    assert(left_motor.last_pi_adj == 0.0f);

    /* Ramp down to complete zero */
    left_motor.cmd_vel = 0.0f;
    for (int i = 0; i < 5; i++) {
        test_time_ms += 10;
        update_control();
    }
    assert(fabsf(left_motor.target_vel) < 1e-4f);

    /* 3. Command timeout check */
    left_motor.cmd_vel = 2.0f;
    right_motor.cmd_vel = 2.0f;
    last_command_ms = test_time_ms;
    for (int i = 0; i < 10; i++) {
        test_time_ms += 10;
        update_control();
    }
    assert(left_motor.target_vel > 1.0f);

    /* Advance time past COMMAND_TIMEOUT_MS (2000 ms) */
    test_time_ms += 2050;
    update_control();
    assert(left_motor.cmd_vel == 0.0f);
    assert(right_motor.cmd_vel == 0.0f);

    /* 4. Direction deadtime coasting */
    left_motor.target_vel = 2.0f;
    left_motor.last_dir_fwd = true;
    set_motor_pwm(&left_motor, 0.5f, false); /* Flip direction to reverse */
    assert(left_motor.last_dir_fwd == false);
    assert(left_motor.dir_change_at_ms == test_time_ms);
}

static void test_bench_vs_motor(void)
{
    reset_test_state();

#if RELOBOT_WHEELS_BENCH
    set_motor_pwm(&left_motor, 0.75f, true);
    set_motor_pwm(&right_motor, 0.75f, true);
    assert(pwm_levels[PIN_LEFT_PWM] == 0);
    assert(pwm_levels[PIN_RIGHT_PWM] == 0);
    assert(!gpio_levels[PIN_LEFT_DIR]);
    assert(!gpio_levels[PIN_RIGHT_DIR]);
#else
    set_motor_pwm(&left_motor, 0.75f, true);
    assert(pwm_levels[PIN_LEFT_PWM] > 0);
    assert(gpio_levels[PIN_LEFT_DIR] == 0); /* FWD: 0 on left DIR */

    set_motor_pwm(&right_motor, 0.75f, true);
    assert(pwm_levels[PIN_RIGHT_PWM] > 0);
    assert(gpio_levels[PIN_RIGHT_DIR] == 1); /* FWD: 1 on right DIR */
#endif
}

static void test_faults(void)
{
    reset_test_state();

    left_motor.cmd_vel = 4.0f;
    right_motor.cmd_vel = 4.0f;
    left_motor.target_vel = 4.0f;
    right_motor.target_vel = 4.0f;

    if (setjmp(fault_return) == 0) {
        relobot_fault();
        assert(false);
    }

    assert(reboot_requested);
    assert(left_motor.cmd_vel == 0.0f);
    assert(left_motor.target_vel == 0.0f);
    assert(right_motor.cmd_vel == 0.0f);
    assert(right_motor.target_vel == 0.0f);

    /* Test Malloc Failed Hook */
    reboot_requested = false;
    if (setjmp(fault_return) == 0) {
        vApplicationMallocFailedHook();
        assert(false);
    }
    assert(reboot_requested);

    /* Test Stack Overflow Hook */
    reboot_requested = false;
    if (setjmp(fault_return) == 0) {
        vApplicationStackOverflowHook(NULL, "test_task");
        assert(false);
    }
    assert(reboot_requested);
}

static void test_cdc(void)
{
    reset_test_state();

    cdc_line_coding_t coding = {.bit_rate = 1200};
    tud_cdc_line_coding_cb(0, &coding);
    assert(reset_usb_boot_called);

    reset_usb_boot_called = false;
    coding.bit_rate = 115200;
    tud_cdc_line_coding_cb(0, &coding);
    assert(!reset_usb_boot_called);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <test_case>\n", argv[0]);
        return 1;
    }

    const char *test = argv[1];
    if (strcmp(test, "defaults") == 0) {
        test_defaults();
    } else if (strcmp(test, "commands") == 0) {
        test_commands();
    } else if (strcmp(test, "control") == 0) {
        test_control();
    } else if (strcmp(test, "bench") == 0 || strcmp(test, "motor") == 0) {
        test_bench_vs_motor();
    } else if (strcmp(test, "faults") == 0) {
        test_faults();
    } else if (strcmp(test, "cdc") == 0) {
        test_cdc();
    } else {
        fprintf(stderr, "Unknown test: %s\n", test);
        return 1;
    }

    printf("Test %s passed!\n", test);
    return 0;
}
