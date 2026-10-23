#include <assert.h>
#include <setjmp.h>
#include <string.h>

#define main firmware_main
#include "../src/main.c"
#undef main

static uint32_t test_time_ms;
static bool enable_levels[32];
static uint16_t pwm_levels[32];
static bool reboot_requested;
static jmp_buf fault_return;

absolute_time_t get_absolute_time(void) { return test_time_ms; }
uint32_t to_ms_since_boot(absolute_time_t time) { return time; }
void sleep_ms(uint32_t milliseconds) { test_time_ms += milliseconds; }
void gpio_put(uint pin, bool value) { enable_levels[pin] = value; }
void pwm_set_gpio_level(uint pin, uint16_t level) { pwm_levels[pin] = level; }
uint32_t save_and_disable_interrupts(void) { return 0; }
void tight_loop_contents(void) { assert(false); }

void watchdog_reboot(uint32_t program_counter, uint32_t stack_pointer, uint32_t delay_ms)
{
    (void)program_counter;
    (void)stack_pointer;
    (void)delay_ms;
    reboot_requested = true;
    longjmp(fault_return, 1);
}

static void assert_outputs_disabled(void)
{
    assert(!enable_levels[PIN_FWD_EN] && !enable_levels[PIN_REV_EN]);
    assert(pwm_levels[PIN_FWD_PWM] == 0 && pwm_levels[PIN_REV_PWM] == 0);
    assert(control_rpm == 0.0f);
    assert(!is_braking);
}

static void prepare_running_motor(void)
{
    pid_controller_init(&rpm_pid, PI_KP, PI_KI, 0.0f,
                        -PI_OUTPUT_LIMIT, PI_OUTPUT_LIMIT,
                        -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
    entities_ready = true;
    last_control_ms = 100;
    last_rpm_sample_ms = 100;
    last_encoder_activity_ms = 100;
    test_time_ms = 100;
    assert(knives_command_accept(&command_state, 0.0f, 100));
    assert(knives_command_accept(&command_state, 1200.0f, 100));
    control_rpm = 1200.0f;
    measured_rpm = 1200.0f;
    set_motor_pwm(0.4f, true);
#if !RELOBOT_KNIVES_BENCH
    assert(enable_levels[PIN_FWD_EN]);
#endif
}

int main(int argument_count, char **arguments)
{
    assert(argument_count == 2);
    prepare_running_motor();
    const char *scenario = arguments[1];
    if (strcmp(scenario, "bench_no_encoder") == 0) {
        assert(RELOBOT_KNIVES_BENCH);
        for (test_time_ms = 120; test_time_ms <= 4120; test_time_ms += CONTROL_PERIOD_MS) {
            update_control();
            assert(command_state.armed);
            assert(control_rpm == 1200.0f);
            assert(!enable_levels[PIN_FWD_EN] && !enable_levels[PIN_REV_EN]);
            assert(pwm_levels[PIN_FWD_PWM] == 0 && pwm_levels[PIN_REV_PWM] == 0);
        }
    } else if (strcmp(scenario, "timeout") == 0) {
        test_time_ms = 5100;
        last_control_ms = 5080;
        last_rpm_sample_ms = 5100;
        is_braking = true;
        update_control();
        assert_outputs_disabled();
        assert(!command_state.armed);
        assert(!knives_command_accept(&command_state, 1200.0f, 5120));
        assert(knives_command_accept(&command_state, 0.0f, 5140));
        assert(knives_command_accept(&command_state, 1200.0f, 5160));
        test_time_ms = 5160;
        update_control();
        assert(enable_levels[PIN_FWD_EN]);
    } else if (strcmp(scenario, "control_gap") == 0) {
        test_time_ms = 201;
        update_control();
        assert_outputs_disabled();
        assert(!command_state.armed);
    } else if (strcmp(scenario, "encoder_loss") == 0) {
        test_time_ms = 2100;
        last_control_ms = 2080;
        last_rpm_sample_ms = 2100;
        update_control();
        assert_outputs_disabled();
        assert(!command_state.armed);
    } else if (strcmp(scenario, "brake_limit") == 0) {
        assert(knives_command_accept(&command_state, 0.0f, 100));
        is_braking = true;
        braking_started_ms = 100;
        test_time_ms = 700;
        last_control_ms = 680;
        last_rpm_sample_ms = 700;
        update_control();
        assert_outputs_disabled();
    } else if (strcmp(scenario, "boot_coasting") == 0) {
        control_rpm = 0.0f;
        assert(knives_command_accept(&command_state, 0.0f, 100));
        test_time_ms = 120;
        update_control();
        assert_outputs_disabled();
    } else if (strcmp(scenario, "coast_telemetry") == 0) {
        test_time_ms = 5100;
        last_control_ms = 5080;
        last_rpm_sample_ms = 4500;
        encoder_ticks = 10;
        update_control();
        assert_outputs_disabled();
        assert(measured_rpm > 0.0f);
        test_time_ms = 5700;
        last_control_ms = 5680;
        update_control();
        assert_outputs_disabled();
        assert(measured_rpm == 0.0f);
    } else {
        if (setjmp(fault_return) == 0) {
            if (strcmp(scenario, "malloc_failure") == 0) {
                vApplicationMallocFailedHook();
            } else {
                assert(strcmp(scenario, "stack_failure") == 0);
                vApplicationStackOverflowHook(NULL, NULL);
            }
            assert(false);
        }
        assert(reboot_requested);
        assert(!enable_levels[PIN_FWD_EN] && !enable_levels[PIN_REV_EN]);
        assert(pwm_levels[PIN_FWD_PWM] == 0 && pwm_levels[PIN_REV_PWM] == 0);
    }
    return 0;
}
