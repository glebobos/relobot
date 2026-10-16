#include <FreeRTOS.h>
#include <task.h>
#include <picoros.h>

#include "command_probe.h"
#include "float32_codec.h"
#include "relobot/board.h"
#include "relobot/picoros_runtime.h"
#include "pico/stdlib.h"
#ifdef PICOROS_CONNECTIVITY
#include "picoros_connectivity.h"
#endif
#ifdef PICOROS_TRANSPORT_PROBE
#include "transport_probe.h"
#endif

#define APP_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 16)
#define APP_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define PROBE_TASK_PRIORITY (APP_TASK_PRIORITY + 1)

static command_probe_t probe;
static TaskHandle_t probe_task_handle;
static TaskHandle_t app_task_handle;
static unsigned int diagnostic_stage;
static bool ready;
static uint32_t publish_errors;
#ifdef PICOROS_CONNECTIVITY
static z_owned_transport_events_listener_t transport_listener;
static bool transport_connected;
static uint32_t transport_opened;
static uint32_t transport_closed;

static void transport_event_received(z_loaned_transport_event_t *event, void *context)
{
    (void)context;
    z_sample_kind_t kind = z_transport_event_kind(event);
    taskENTER_CRITICAL();
    if (kind == Z_SAMPLE_KIND_PUT) {
        transport_connected = true;
        transport_opened++;
    } else if (kind == Z_SAMPLE_KIND_DELETE) {
        transport_connected = false;
        transport_closed++;
    }
    taskEXIT_CRITICAL();
}
#endif
#ifdef PICOROS_DIAGNOSTICS
static uint32_t last_app_progress_ms;
#endif

static picoros_node_t node = {.name = "picoros_usb_bidirectional"};
static picoros_publisher_t publishers[] = {
    {.topic = {.name = "picoros_usb_spike/value", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/ack", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/active", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/rejected", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/invalid", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/timeouts", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/publish_errors", .type = "std_msgs::msg::dds_::Float32"}},
#ifdef PICOROS_CONNECTIVITY
    {.topic = {.name = "picoros_usb_spike/transport_connected", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/transport_opened", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/transport_closed", .type = "std_msgs::msg::dds_::Float32"}},
#endif
    {.topic = {.name = "picoros_usb_spike/heap_free", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/heap_min_ever", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/stack_app_words", .type = "std_msgs::msg::dds_::Float32"}},
    {.topic = {.name = "picoros_usb_spike/stack_probe_words", .type = "std_msgs::msg::dds_::Float32"}},
};

void picoros_diagnostic_stage(unsigned int stage)
{
    taskENTER_CRITICAL();
    diagnostic_stage = stage;
    taskEXIT_CRITICAL();
}

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void command_received(uint8_t *payload, size_t length)
{
    float value;
    bool decoded = float32_codec_decode(payload, length, &value);
    taskENTER_CRITICAL();
    if (decoded) {
        command_probe_receive(&probe, value, now_ms());
    } else {
        probe.invalid++;
    }
    taskEXIT_CRITICAL();
}

static picoros_subscriber_t subscriber = {
    .topic = {.name = "picoros_usb_spike/command", .type = "std_msgs::msg::dds_::Float32"},
    .user_callback = command_received,
};

static void fail(unsigned int flashes)
{
    taskENTER_CRITICAL();
    diagnostic_stage = 100 + flashes;
    taskEXIT_CRITICAL();
    vTaskSuspend(NULL);
    for (;;) {
        tight_loop_contents();
    }
}

static void probe_task(void *argument)
{
    (void)argument;
    TickType_t previous_wake = xTaskGetTickCount();
    for (;;) {
        taskENTER_CRITICAL();
        command_probe_tick(&probe, now_ms());
        bool active = probe.active;
        bool initialized = ready;
        unsigned int stage = diagnostic_stage;
#ifdef PICOROS_DIAGNOSTICS
        uint32_t app_progress_ms = last_app_progress_ms;
#endif
#ifdef PICOROS_TRANSPORT_PROBE
        transport_probe_status_t transport_status = transport_probe_status();
#endif
        taskEXIT_CRITICAL();

        if (stage >= 100) {
            unsigned int phase = (now_ms() % (300 * (stage - 100) + 1000)) / 150;
            relobot_board_led_set(phase < 2 * (stage - 100) && phase % 2 == 0 ? 32 : 0, 0, 0);
        }
#ifdef PICOROS_DIAGNOSTICS
        else if (now_ms() % 1000 < 100) {
            relobot_board_led_set(0, 0, 0);
        } else if (initialized && (uint32_t)(now_ms() - app_progress_ms) >= 2000) {
            relobot_board_led_set(24, 0, 24);
        }
#endif
#ifdef PICOROS_TRANSPORT_PROBE
        else if (initialized && transport_status == TRANSPORT_PROBE_SERIAL_WAIT) {
            relobot_board_led_set(0, 24, 24);
        } else if (initialized && transport_status == TRANSPORT_PROBE_NO_EXECUTOR_PROGRESS) {
            relobot_board_led_set(24, 24, 24);
        }
#endif
        else if (initialized) {
            relobot_board_led_set(active ? 0 : 24, active ? 24 : 12, 0);
        } else {
            relobot_board_led_set(0, 0, stage % 2 == 0 ? 24 : 8);
        }
        vTaskDelayUntil(&previous_wake, pdMS_TO_TICKS(10));
    }
}

static void app_task(void *argument)
{
    (void)argument;
    if (!float32_codec_init()) {
        fail(1);
    }
    subscriber.topic.rihs_hash = float32_codec_hash();
    picoros_res_t status = relobot_picoros_init();
    if (status != PICOROS_OK) {
        fail(2);
    }
#ifdef PICOROS_CONNECTIVITY
    if (!picoros_connectivity_observe(&transport_listener, transport_event_received, NULL)) {
        fail(7);
    }
#endif
    if (picoros_node_init(&node) != PICOROS_OK) {
        fail(3);
    }
    for (size_t index = 0; index < sizeof(publishers) / sizeof(publishers[0]); ++index) {
        publishers[index].topic.rihs_hash = float32_codec_hash();
        z_publisher_options_default(&publishers[index].opts);
        if (picoros_publisher_declare(&node, &publishers[index]) != PICOROS_OK) {
            fail(4);
        }
    }
    if (picoros_subscriber_declare(&node, &subscriber) != PICOROS_OK) {
        fail(5);
    }
    taskENTER_CRITICAL();
    ready = true;
    taskEXIT_CRITICAL();

    for (;;) {
        taskENTER_CRITICAL();
        command_probe_t snapshot = probe;
#ifdef PICOROS_CONNECTIVITY
        bool connected = transport_connected;
        uint32_t opened = transport_opened;
        uint32_t closed = transport_closed;
#endif
#ifdef PICOROS_DIAGNOSTICS
        last_app_progress_ms = now_ms();
#endif
        taskEXIT_CRITICAL();
        float values[] = {
            (float)now_ms() / 1000.0f, (float)snapshot.last_sequence,
            snapshot.active ? 1.0f : 0.0f, (float)snapshot.rejected,
            (float)snapshot.invalid, (float)snapshot.timeouts, (float)publish_errors,
#ifdef PICOROS_CONNECTIVITY
            connected ? 1.0f : 0.0f, (float)opened, (float)closed,
#endif
            (float)xPortGetFreeHeapSize(),
            (float)xPortGetMinimumEverFreeHeapSize(),
            (float)uxTaskGetStackHighWaterMark(NULL),
            probe_task_handle != NULL ? (float)uxTaskGetStackHighWaterMark(probe_task_handle) : 0.0f,
        };
        _Static_assert(sizeof(values) / sizeof(values[0]) == sizeof(publishers) / sizeof(publishers[0]),
                       "values array size must match publishers array size");
        for (size_t index = 0; index < sizeof(publishers) / sizeof(publishers[0]); ++index) {
            uint8_t payload[FLOAT32_CDR_SIZE];
            if (!float32_codec_encode(values[index], payload)) {
                fail(6);
            }
            if (picoros_publish(&publishers[index], payload, sizeof(payload)) != PICOROS_OK) {
                publish_errors++;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void)
{
    relobot_board_init();
    relobot_board_led_set(32, 0, 32);
    if (xTaskCreate(probe_task, "command_probe", configMINIMAL_STACK_SIZE,
                    NULL, PROBE_TASK_PRIORITY, &probe_task_handle) != pdPASS ||
        xTaskCreate(app_task, "picoros_usb", APP_TASK_STACK_WORDS,
                    NULL, APP_TASK_PRIORITY, &app_task_handle) != pdPASS) {
        relobot_board_led_set(32, 0, 0);
        for (;;) {
            tight_loop_contents();
        }
    }
    vTaskStartScheduler();
    relobot_board_led_set(32, 0, 0);
    for (;;) {
        tight_loop_contents();
    }
}
