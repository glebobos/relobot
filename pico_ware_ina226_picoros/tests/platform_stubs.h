#ifndef INA226_PLATFORM_STUBS_H
#define INA226_PLATFORM_STUBS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef uint32_t absolute_time_t;
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
#define BOOL_CDR_SIZE 8u

extern i2c_inst_t *const i2c1;

absolute_time_t get_absolute_time(void);
uint32_t to_ms_since_boot(absolute_time_t time);
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
int i2c_write_blocking_until(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop, absolute_time_t until);
int i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, absolute_time_t until);
absolute_time_t make_timeout_time_ms(uint32_t ms);

bool float32_codec_init(void);
const char *float32_codec_hash(void);
bool float32_codec_encode(float value, uint8_t payload[FLOAT32_CDR_SIZE]);
bool float32_codec_decode(uint8_t *payload, size_t length, float *value);

bool bool_codec_init(void);
const char *bool_codec_hash(void);
bool bool_codec_encode(bool value, uint8_t payload[BOOL_CDR_SIZE], size_t *actual_length);
bool bool_codec_decode(const uint8_t *payload, size_t length, bool *value);

#endif /* INA226_PLATFORM_STUBS_H */
