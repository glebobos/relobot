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
static absolute_time_t test_time_ms = 0;

static int mock_i2c_write_ret = 1;
static int mock_i2c_read_ret = 2;
static uint8_t mock_i2c_read_data[2] = {0x2E, 0xE0}; /* 12000 = 15.0V */
static void *mock_i2c1_inst = (void *)0x1234;
i2c_inst_t *const i2c1 = (i2c_inst_t *)&mock_i2c1_inst;

/* Platform stubs implementation */
absolute_time_t get_absolute_time(void) { return test_time_ms; }
uint32_t to_ms_since_boot(absolute_time_t time) { return time; }
void sleep_ms(uint32_t ms) { test_time_ms += ms; }
uint32_t save_and_disable_interrupts(void) { return 0; }
void tight_loop_contents(void) { assert(false); }

void gpio_init(uint pin) { (void)pin; }
void gpio_set_dir(uint pin, bool output) { (void)pin; (void)output; }
void gpio_put(uint pin, bool value) { (void)pin; (void)value; }
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
void gpio_pull_up(uint pin) { (void)pin; }

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

void watchdog_enable(uint32_t delay_ms, bool pause) { (void)delay_ms; (void)pause; }
void watchdog_update(void) {}

TickType_t xTaskGetTickCount(void) { return test_time_ms; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
void vTaskDelayUntil(TickType_t *previous, TickType_t ticks) { (void)previous; (void)ticks; }
int xTaskCreate(void (*task)(void *), const char *name, uint32_t depth,
                void *argument, unsigned int priority, TaskHandle_t *handle) {
    (void)task; (void)name; (void)depth; (void)argument; (void)priority; (void)handle;
    return pdPASS;
}
void vTaskStartScheduler(void) {}

size_t xPortGetFreeHeapSize(void) { return 64000; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return 60000; }
unsigned int uxTaskGetStackHighWaterMark(TaskHandle_t task) { (void)task; return 256; }

picoros_res_t picoros_node_init(picoros_node_t *n) { (void)n; return PICOROS_OK; }
picoros_res_t picoros_publisher_declare(picoros_node_t *n, picoros_publisher_t *p) { (void)n; (void)p; return PICOROS_OK; }
picoros_res_t picoros_publish(picoros_publisher_t *p, const uint8_t *payload, size_t length) { (void)p; (void)payload; (void)length; return PICOROS_OK; }
void z_publisher_options_default(int *opts) { (void)opts; }

void relobot_board_init(void) {}
void relobot_board_led_set(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
picoros_res_t relobot_picoros_init(void) { return PICOROS_OK; }

int i2c_init(i2c_inst_t *i2c, uint baudrate) { (void)i2c; (void)baudrate; return 0; }
absolute_time_t make_timeout_time_ms(uint32_t ms) { return test_time_ms + ms; }

int i2c_write_blocking_until(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop, absolute_time_t until)
{
    (void)i2c; (void)addr; (void)src; (void)len; (void)nostop; (void)until;
    return mock_i2c_write_ret;
}

int i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, absolute_time_t until)
{
    (void)i2c; (void)addr; (void)nostop; (void)until;
    if (mock_i2c_read_ret < 0) return mock_i2c_read_ret;
    if (dst && len >= 2) {
        dst[0] = mock_i2c_read_data[0];
        dst[1] = mock_i2c_read_data[1];
    }
    return mock_i2c_read_ret;
}

bool float32_codec_init(void) { return true; }
const char *float32_codec_hash(void) { return "1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef"; }
bool float32_codec_encode(float value, uint8_t payload[FLOAT32_CDR_SIZE]) { (void)value; (void)payload; return true; }
bool float32_codec_decode(uint8_t *payload, size_t length, float *value) { (void)payload; (void)length; *value = 0.0f; return true; }

bool bool_codec_init(void) { return true; }
const char *bool_codec_hash(void) { return "abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890"; }
bool bool_codec_encode(bool value, uint8_t payload[BOOL_CDR_SIZE], size_t *actual_length) {
    (void)value; (void)payload;
    if (actual_length) *actual_length = 5;
    return true;
}
bool bool_codec_decode(const uint8_t *payload, size_t length, bool *value) {
    (void)payload; (void)length; *value = true; return true;
}

static void test_voltage_conversion(void)
{
    assert(fabsf(ina226_raw_to_voltage(0) - 0.0f) < 1e-6f);
    assert(fabsf(ina226_raw_to_voltage(8000) - 10.0f) < 1e-5f);
    assert(fabsf(ina226_raw_to_voltage(10000) - 12.5f) < 1e-5f);
    assert(fabsf(ina226_raw_to_voltage(11840) - 14.8f) < 1e-5f);
}

static void test_dock_detection(void)
{
    assert(!ina226_is_on_dock(0.0f));
    assert(!ina226_is_on_dock(5.0f));
    assert(!ina226_is_on_dock(12.0f));
    assert(!ina226_is_on_dock(12.5f));

    assert(ina226_is_on_dock(12.51f));
    assert(ina226_is_on_dock(14.0f));
    assert(ina226_is_on_dock(19.0f));

    assert(!ina226_is_on_dock(NAN));
    assert(!ina226_is_on_dock(-1.0f));
}

static void test_bench_readings(void)
{
#if RELOBOT_INA226_BENCH
    float v_bat = ina226_read_bus_voltage(INA226_ADDR_BATTERY);
    float v_chg = ina226_read_bus_voltage(INA226_ADDR_CHARGER);
    float v_inv = ina226_read_bus_voltage(0x99);

    assert(!isnan(v_bat) && v_bat > 14.0f && v_bat < 15.0f);
    assert(!isnan(v_chg) && fabsf(v_chg - 0.0f) < 1e-5f);
    assert(!ina226_is_on_dock(v_chg));
    assert(isnan(v_inv));
#endif
}

static void test_sensor_readings(void)
{
#if !RELOBOT_INA226_BENCH
    ina226_hardware_init();
    /* Success read: raw 12000 -> 15.0V */
    mock_i2c_write_ret = 1;
    mock_i2c_read_ret = 2;
    mock_i2c_read_data[0] = (12000 >> 8) & 0xFF;
    mock_i2c_read_data[1] = 12000 & 0xFF;

    float v_read = ina226_read_bus_voltage(INA226_ADDR_BATTERY);
    assert(!isnan(v_read));
    assert(fabsf(v_read - 15.0f) < 1e-5f);

    /* I2C write error: returns NAN */
    mock_i2c_write_ret = -1;
    assert(isnan(ina226_read_bus_voltage(INA226_ADDR_BATTERY)));

    /* I2C read error: returns NAN */
    mock_i2c_write_ret = 1;
    mock_i2c_read_ret = -1;
    assert(isnan(ina226_read_bus_voltage(INA226_ADDR_BATTERY)));
#endif
}

static void test_fault_hooks(void)
{
    reboot_requested = false;
    if (setjmp(fault_return) == 0) {
        relobot_fault();
        assert(false);
    }
    assert(reboot_requested);

    reboot_requested = false;
    if (setjmp(fault_return) == 0) {
        vApplicationMallocFailedHook();
        assert(false);
    }
    assert(reboot_requested);

    reboot_requested = false;
    if (setjmp(fault_return) == 0) {
        vApplicationStackOverflowHook(NULL, "test_task");
        assert(false);
    }
    assert(reboot_requested);
}

static void test_cdc_1200_touch(void)
{
    reset_usb_boot_called = false;
    cdc_line_coding_t normal_coding = {.bit_rate = 115200};
    tud_cdc_line_coding_cb(0, &normal_coding);
    assert(!reset_usb_boot_called);

    cdc_line_coding_t boot_coding = {.bit_rate = 1200};
    tud_cdc_line_coding_cb(0, &boot_coding);
    assert(reset_usb_boot_called);
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        if (strcmp(argv[1], "voltage") == 0) {
            test_voltage_conversion();
            return 0;
        } else if (strcmp(argv[1], "dock") == 0) {
            test_dock_detection();
            return 0;
        } else if (strcmp(argv[1], "bench") == 0) {
            test_bench_readings();
            return 0;
        } else if (strcmp(argv[1], "sensor") == 0) {
            test_sensor_readings();
            return 0;
        } else if (strcmp(argv[1], "faults") == 0) {
            test_fault_hooks();
            return 0;
        } else if (strcmp(argv[1], "cdc") == 0) {
            test_cdc_1200_touch();
            return 0;
        }
    }

    test_voltage_conversion();
    test_dock_detection();
    test_bench_readings();
    test_sensor_readings();
    test_fault_hooks();
    test_cdc_1200_touch();

    return 0;
}
