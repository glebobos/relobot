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
static absolute_time_t test_time_us = 0;

static void *mock_i2c1_inst = (void *)0x1234;
i2c_inst_t *const i2c1 = (i2c_inst_t *)&mock_i2c1_inst;

/* Platform stubs implementation */
absolute_time_t get_absolute_time(void) { return test_time_us; }
uint32_t to_ms_since_boot(absolute_time_t time) { return (uint32_t)(time / 1000); }
uint64_t to_us_since_boot(absolute_time_t time) { return time; }
void sleep_ms(uint32_t ms) { test_time_us += ((uint64_t)ms * 1000); }
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

TickType_t xTaskGetTickCount(void) { return (TickType_t)(test_time_us / 1000); }
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
picoros_res_t picoros_node_drop(picoros_node_t *n) { (void)n; return PICOROS_OK; }
picoros_res_t picoros_publisher_declare(picoros_node_t *n, picoros_publisher_t *p) { (void)n; (void)p; return PICOROS_OK; }
picoros_res_t picoros_publisher_drop(picoros_publisher_t *p) { (void)p; return PICOROS_OK; }
picoros_res_t picoros_publish(picoros_publisher_t *p, const uint8_t *payload, size_t length) { (void)p; (void)payload; (void)length; return PICOROS_OK; }
void picoros_interface_close(void) {}
void z_publisher_options_default(int *opts) { (void)opts; }

void relobot_board_init(void) {}
void relobot_board_led_set(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
picoros_res_t relobot_picoros_init(void) { return PICOROS_OK; }

int i2c_init(i2c_inst_t *i2c, uint baudrate) { (void)i2c; (void)baudrate; return 0; }

bool float32_codec_init(void) { return true; }
const char *float32_codec_hash(void) { return "1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef"; }
bool float32_codec_encode(float value, uint8_t payload[FLOAT32_CDR_SIZE]) { (void)value; (void)payload; return true; }
bool float32_codec_decode(uint8_t *payload, size_t length, float *value) { (void)payload; (void)length; *value = 0.0f; return true; }

bool imu_codec_init(void) { return true; }
const char *imu_codec_hash(void) { return "7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b"; }
bool imu_codec_encode(const sensor_msgs__msg__Imu *msg, uint8_t *payload, size_t max_len, size_t *actual_length)
{
    (void)msg; (void)payload; (void)max_len;
    if (actual_length) *actual_length = 328;
    return true;
}
bool imu_codec_decode(const uint8_t *payload, size_t length, sensor_msgs__msg__Imu *msg)
{
    (void)payload; (void)length; (void)msg;
    return true;
}

void imu_msg_init_defaults(sensor_msgs__msg__Imu *msg, char *frame_id_buf, size_t frame_id_buf_size)
{
    if (msg == NULL) return;
    memset(msg, 0, sizeof(*msg));

    if (frame_id_buf != NULL) {
        msg->header.frame_id.data = frame_id_buf;
        msg->header.frame_id.size = strlen(frame_id_buf);
        msg->header.frame_id.capacity = frame_id_buf_size;
    }

    msg->orientation_covariance[0] = 0.001;
    msg->orientation_covariance[4] = 0.001;
    msg->orientation_covariance[8] = 0.001;

    msg->angular_velocity_covariance[0] = 0.0001;
    msg->angular_velocity_covariance[4] = 0.0001;
    msg->angular_velocity_covariance[8] = 0.0001;

    msg->linear_acceleration_covariance[0] = 0.01;
    msg->linear_acceleration_covariance[4] = 0.01;
    msg->linear_acceleration_covariance[8] = 0.01;

    msg->orientation.w = 1.0;
}

static void test_imu_defaults(void)
{
    sensor_msgs__msg__Imu msg;
    char buf[16] = "imu_link";
    imu_msg_init_defaults(&msg, buf, sizeof(buf));

    assert(strcmp(msg.header.frame_id.data, "imu_link") == 0);
    assert(msg.header.frame_id.size == 8);

    /* Orientation covariance */
    assert(fabs(msg.orientation_covariance[0] - 0.001) < 1e-6);
    assert(fabs(msg.orientation_covariance[4] - 0.001) < 1e-6);
    assert(fabs(msg.orientation_covariance[8] - 0.001) < 1e-6);
    assert(fabs(msg.orientation_covariance[1] - 0.0) < 1e-6);

    /* Angular velocity covariance */
    assert(fabs(msg.angular_velocity_covariance[0] - 0.0001) < 1e-6);
    assert(fabs(msg.angular_velocity_covariance[4] - 0.0001) < 1e-6);
    assert(fabs(msg.angular_velocity_covariance[8] - 0.0001) < 1e-6);

    /* Linear acceleration covariance */
    assert(fabs(msg.linear_acceleration_covariance[0] - 0.01) < 1e-6);
    assert(fabs(msg.linear_acceleration_covariance[4] - 0.01) < 1e-6);
    assert(fabs(msg.linear_acceleration_covariance[8] - 0.01) < 1e-6);

    /* Identity quaternion */
    assert(fabs(msg.orientation.w - 1.0) < 1e-6);
    assert(fabs(msg.orientation.x - 0.0) < 1e-6);
    assert(fabs(msg.orientation.y - 0.0) < 1e-6);
    assert(fabs(msg.orientation.z - 0.0) < 1e-6);
}

static void test_bench_readings(void)
{
#if RELOBOT_IMU_BENCH
    imu_sensor_data_t data;
    bool ok = imu_sensor_read_data(&data);
    assert(ok);
    assert(data.orientation_valid);
    assert(fabsf(data.orientation[0] - 1.0f) < 1e-5f); /* qw */
    assert(fabsf(data.orientation[1] - 0.0f) < 1e-5f); /* qx */
    assert(fabsf(data.orientation[2] - 0.0f) < 1e-5f); /* qy */
    assert(fabsf(data.orientation[3] - 0.0f) < 1e-5f); /* qz */

    assert(data.angular_velocity_valid);
    assert(fabsf(data.angular_velocity[0] - 0.0f) < 1e-5f);
    assert(fabsf(data.angular_velocity[1] - 0.0f) < 1e-5f);
    assert(fabsf(data.angular_velocity[2] - 0.0f) < 1e-5f);

    assert(data.linear_acceleration_valid);
    assert(fabsf(data.linear_acceleration[0] - 0.0f) < 1e-5f);
    assert(fabsf(data.linear_acceleration[1] - 0.0f) < 1e-5f);
    assert(fabsf(data.linear_acceleration[2] - 9.80665f) < 1e-4f);
#endif
}

static void test_sensor_readings(void)
{
#if !RELOBOT_IMU_BENCH
    imu_sensor_data_t data;
    /* In unit test mode without physical sensor, returns false */
    bool ok = imu_sensor_read_data(&data);
    (void)ok;
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

static void test_data_staleness(void)
{
    uint32_t last_ms = 1000;

    /* A fresh reading resets the staleness window */
    assert(!update_freshness(&last_ms, true, 1000, IMU_STALE_THRESHOLD_MS));
    assert(last_ms == 1000);

    /* No update yet, but still inside the threshold: not stale */
    assert(!update_freshness(&last_ms, false, 1000 + IMU_STALE_THRESHOLD_MS - 1, IMU_STALE_THRESHOLD_MS));
    assert(last_ms == 1000);

    /* No update at/after the threshold: stale */
    assert(update_freshness(&last_ms, false, 1000 + IMU_STALE_THRESHOLD_MS, IMU_STALE_THRESHOLD_MS));

    /* A later fresh reading clears staleness again */
    assert(!update_freshness(&last_ms, true, 50000, IMU_STALE_THRESHOLD_MS));
    assert(last_ms == 50000);
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
        if (strcmp(argv[1], "defaults") == 0) {
            test_imu_defaults();
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
        } else if (strcmp(argv[1], "staleness") == 0) {
            test_data_staleness();
            return 0;
        }
    }

    test_imu_defaults();
    test_bench_readings();
    test_sensor_readings();
    test_fault_hooks();
    test_cdc_1200_touch();
    test_data_staleness();

    return 0;
}
