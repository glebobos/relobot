#ifndef IMU_PLATFORM_STUBS_H
#define IMU_PLATFORM_STUBS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef uint64_t absolute_time_t;
typedef struct { uint32_t bit_rate; } cdc_line_coding_t;
typedef enum { PICOROS_OK, PICOROS_ERROR, PICOROS_NOT_READY } picoros_res_t;
typedef struct { const char *name; } picoros_node_t;
typedef struct { const char *name; const char *type; const char *rihs_hash; } test_topic_t;
typedef struct { test_topic_t topic; int opts; } picoros_publisher_t;
typedef void *i2c_inst_t;

#define configMINIMAL_STACK_SIZE 512
#define tskIDLE_PRIORITY 0
#define pdPASS 1
#define pdMS_TO_TICKS(milliseconds) (milliseconds)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define GPIO_OUT 1
#define GPIO_IN 0
#define GPIO_FUNC_I2C 3

#define FLOAT32_CDR_SIZE 8u
#ifndef IMU_CDR_MAX_SIZE
#define IMU_CDR_MAX_SIZE 512
#endif

/* ROS 2 message stubs for unit test environment */
typedef struct {
    int32_t sec;
    uint32_t nanosec;
} builtin_interfaces__msg__Time;

typedef struct {
    char *data;
    size_t size;
    size_t capacity;
} rosidl_runtime_c__String;

typedef struct {
    builtin_interfaces__msg__Time stamp;
    rosidl_runtime_c__String frame_id;
} std_msgs__msg__Header;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} geometry_msgs__msg__Quaternion;

typedef struct {
    double x;
    double y;
    double z;
} geometry_msgs__msg__Vector3;

typedef struct {
    std_msgs__msg__Header header;
    geometry_msgs__msg__Quaternion orientation;
    double orientation_covariance[9];
    geometry_msgs__msg__Vector3 angular_velocity;
    double angular_velocity_covariance[9];
    geometry_msgs__msg__Vector3 linear_acceleration;
    double linear_acceleration_covariance[9];
} sensor_msgs__msg__Imu;

extern i2c_inst_t *const i2c1;

absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t time);
uint64_t to_us_since_boot(absolute_time_t time);
void sleep_ms(uint32_t milliseconds);
void tight_loop_contents(void);
void reset_usb_boot(uint32_t gpio_mask, uint32_t disable_mask);
void gpio_init(uint pin);
void gpio_set_dir(uint pin, bool output);
void gpio_put(uint pin, bool value);
void gpio_set_function(uint pin, uint function);
void gpio_pull_up(uint pin);
uint32_t save_and_disable_interrupts(void);
void watchdog_reboot(uint32_t program_counter, uint32_t stack_pointer, uint32_t delay_ms);
void watchdog_enable(uint32_t delay_ms, bool pause_on_debug);
void watchdog_update(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *previous, TickType_t ticks);
int xTaskCreate(void (*task)(void *), const char *name, uint32_t depth,
                void *argument, unsigned int priority, TaskHandle_t *handle);
void vTaskStartScheduler(void);
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);
unsigned int uxTaskGetStackHighWaterMark(TaskHandle_t task);
picoros_res_t picoros_node_init(picoros_node_t *node);
picoros_res_t picoros_publisher_declare(picoros_node_t *node, picoros_publisher_t *publisher);
picoros_res_t picoros_publish(picoros_publisher_t *publisher, const uint8_t *payload, size_t length);
void z_publisher_options_default(int *options);
void relobot_board_init(void);
void relobot_board_led_set(uint8_t r, uint8_t g, uint8_t b);
picoros_res_t relobot_picoros_init(void);

int i2c_init(i2c_inst_t *i2c, uint baudrate);

bool float32_codec_init(void);
const char *float32_codec_hash(void);
bool float32_codec_encode(float value, uint8_t payload[FLOAT32_CDR_SIZE]);
bool float32_codec_decode(uint8_t *payload, size_t length, float *value);

bool imu_codec_init(void);
const char *imu_codec_hash(void);
bool imu_codec_encode(const sensor_msgs__msg__Imu *msg, uint8_t *payload, size_t max_len, size_t *actual_length);
bool imu_codec_decode(const uint8_t *payload, size_t length, sensor_msgs__msg__Imu *msg);
void imu_msg_init_defaults(sensor_msgs__msg__Imu *msg, char *frame_id_buf, size_t frame_id_buf_size);

#endif /* IMU_PLATFORM_STUBS_H */
