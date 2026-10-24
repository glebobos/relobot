#ifndef WHEELS_PLATFORM_STUBS_H
#define WHEELS_PLATFORM_STUBS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef uint64_t absolute_time_t;
typedef struct { unsigned int unused; } pwm_config;
typedef struct { uint32_t bit_rate; } cdc_line_coding_t;
typedef enum { PICOROS_OK, PICOROS_ERROR, PICOROS_NOT_READY } picoros_res_t;
typedef struct { const char *name; } picoros_node_t;
typedef struct { const char *name; const char *type; const char *rihs_hash; } test_topic_t;
typedef struct { test_topic_t topic; int opts; } picoros_publisher_t;
typedef struct { test_topic_t topic; void (*user_callback)(uint8_t *, size_t); } picoros_subscriber_t;

#define configMINIMAL_STACK_SIZE 512
#define tskIDLE_PRIORITY 0
#define pdPASS 1
#define pdMS_TO_TICKS(milliseconds) (milliseconds)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define GPIO_OUT 1
#define GPIO_IN 0
#define GPIO_FUNC_PWM 4
#define GPIO_IRQ_EDGE_RISE 8

#define FLOAT32_CDR_SIZE 8u
#ifndef JOINT_STATE_CDR_MAX_SIZE
#define JOINT_STATE_CDR_MAX_SIZE 512
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
    rosidl_runtime_c__String *data;
    size_t size;
    size_t capacity;
} rosidl_runtime_c__String__Sequence;

typedef struct {
    double *data;
    size_t size;
    size_t capacity;
} rosidl_runtime_c__double__Sequence;

typedef struct {
    builtin_interfaces__msg__Time stamp;
    rosidl_runtime_c__String frame_id;
} std_msgs__msg__Header;

typedef struct {
    std_msgs__msg__Header header;
    rosidl_runtime_c__String__Sequence name;
    rosidl_runtime_c__double__Sequence position;
    rosidl_runtime_c__double__Sequence velocity;
    rosidl_runtime_c__double__Sequence effort;
} sensor_msgs__msg__JointState;

typedef struct {
    float data;
} std_msgs__msg__Float32;

absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t time);
uint64_t to_us_since_boot(absolute_time_t time);
uint32_t time_us_32(void);
void sleep_ms(uint32_t milliseconds);
void tight_loop_contents(void);
void reset_usb_boot(uint32_t gpio_mask, uint32_t disable_mask);
bool gpio_get(uint pin);
void busy_wait_us_32(uint32_t delay_us);
void gpio_init(uint pin);
void gpio_set_dir(uint pin, bool output);
void gpio_put(uint pin, bool value);
void gpio_set_function(uint pin, uint function);
void gpio_pull_up(uint pin);
void gpio_set_irq_enabled(uint pin, uint32_t events, bool enabled);
void gpio_set_irq_enabled_with_callback(uint pin, uint32_t events, bool enabled,
                                       void (*callback)(uint, uint32_t));
void pwm_set_gpio_level(uint pin, uint16_t level);
uint pwm_gpio_to_slice_num(uint pin);
pwm_config pwm_get_default_config(void);
void pwm_config_set_wrap(pwm_config *config, uint16_t wrap);
void pwm_init(uint slice, const pwm_config *config, bool enabled);
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
picoros_res_t picoros_node_drop(picoros_node_t *node);
picoros_res_t picoros_publisher_declare(picoros_node_t *node, picoros_publisher_t *publisher);
picoros_res_t picoros_publisher_drop(picoros_publisher_t *publisher);
picoros_res_t picoros_subscriber_declare(picoros_node_t *node, picoros_subscriber_t *subscriber);
picoros_res_t picoros_subscriber_drop(picoros_subscriber_t *subscriber);
picoros_res_t picoros_publish(picoros_publisher_t *publisher, const uint8_t *payload, size_t length);
void picoros_interface_close(void);
void z_publisher_options_default(int *options);

#endif /* WHEELS_PLATFORM_STUBS_H */
