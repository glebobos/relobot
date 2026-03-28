#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico_uart_transports.h"

// ---- pin / hardware --------------------------------------------------------
#define I2C_SDA_PIN             6
#define I2C_SCL_PIN             7
#define I2C_BAUD_RATE           400000
#define I2C_TIMEOUT_MS          5
#define LED_PIN                 PICO_DEFAULT_LED_PIN

// ---- INA226 ----------------------------------------------------------------
#define INA226_ADDR_BATTERY     0x40    // battery pack
#define INA226_ADDR_CHARGER     0x41    // docking station charger
#define INA226_REG_BUS_VOLTAGE  0x02

// ---- docking ---------------------------------------------------------------
// Voltage on the charger channel above which the robot is considered on-dock.
// TODO: tune to actual charging system voltage once hardware is characterised.
#define DOCK_VOLTAGE_THRESHOLD  12.5f

// ---- timing / micro-ROS agent ----------------------------------------------
#define TIMER_PERIOD_MS         100     // 10 Hz publish rate
#define AGENT_POLL_MS           500     // ping interval when waiting for agent
#define AGENT_KEEPALIVE_MS      1000    // ping interval when connected
#define AGENT_PING_TIMEOUT_MS   100     // per-ping timeout
#define AGENT_PING_ATTEMPTS     1       // pings per check
#define TIME_SYNC_TIMEOUT_MS    1000    // rmw session sync timeout
#define WATCHDOG_TIMEOUT_MS     2000    // reset device if loop stalls

// ---------------------------------------------------------------------------
// Context struct — groups all micro-ROS handles
// ---------------------------------------------------------------------------
typedef struct {
    rcl_publisher_t  pub_battery;
    rcl_publisher_t  pub_charger;
    rcl_publisher_t  pub_on_dock;
    rcl_timer_t      timer;
    rcl_node_t       node;
    rcl_allocator_t  allocator;
    rclc_support_t   support;
    rclc_executor_t  executor;
} app_context_t;

static app_context_t ctx;

static std_msgs__msg__Float32 msg_battery;
static std_msgs__msg__Float32 msg_charger;
static std_msgs__msg__Bool    msg_on_dock;

#define EXECUTE_EVERY_N_MS(MS, X) do {                          \
    static int64_t init = -1;                                   \
    if (init == -1) { init = (int64_t)uxr_millis(); }           \
    if ((int64_t)uxr_millis() - init > (MS)) {                  \
        X;                                                      \
        init = (int64_t)uxr_millis();                           \
    }                                                           \
} while (0)

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// ---------------------------------------------------------------------------
// LED helper
// ---------------------------------------------------------------------------
static void set_state_led(enum states s)
{
    gpio_put(LED_PIN, s == AGENT_CONNECTED);
}

// ---------------------------------------------------------------------------
// INA226 helpers
// ---------------------------------------------------------------------------
/**
 * Read the INA226 bus voltage register (0x02).
 * Returns voltage in Volts (1.25 mV per LSB), or NAN on any I2C error.
 * Uses timeout variants to avoid hanging on a faulted bus.
 */
static float ina226_read_bus_voltage(i2c_inst_t *i2c, uint8_t addr)
{
    uint8_t reg = INA226_REG_BUS_VOLTAGE;
    int wrc = i2c_write_blocking_until(i2c, addr, &reg, 1, true,
                  make_timeout_time_ms(I2C_TIMEOUT_MS));
    if (wrc < 0) return NAN;

    uint8_t buf[2] = {0, 0};
    int rrc = i2c_read_blocking_until(i2c, addr, buf, 2, false,
                  make_timeout_time_ms(I2C_TIMEOUT_MS));
    if (rrc < 0) return NAN;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return raw * 0.00125f;
}

// ---------------------------------------------------------------------------
// micro-ROS timer callback
// ---------------------------------------------------------------------------
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    float v_battery = ina226_read_bus_voltage(i2c1, INA226_ADDR_BATTERY);
    float v_charger = ina226_read_bus_voltage(i2c1, INA226_ADDR_CHARGER);

    // on_dock: explicitly guard against NAN so the check is never accidentally
    // correct — charger must be a valid reading AND above threshold.
    // TODO: replace with richer docking logic when docking station interface
    //       is finalised (e.g. combined contact detection + voltage check).
    msg_on_dock.data = !isnan(v_charger) && (v_charger > DOCK_VOLTAGE_THRESHOLD);

    // Skip invalid readings so subscribers never see I2C error values as data.
    if (!isnan(v_battery)) {
        msg_battery.data = v_battery;
        rcl_publish(&ctx.pub_battery, &msg_battery, NULL);
    }
    if (!isnan(v_charger)) {
        msg_charger.data = v_charger;
        rcl_publish(&ctx.pub_charger, &msg_charger, NULL);
    }

    // Always publish dock status — false on I2C error is the safe default.
    rcl_publish(&ctx.pub_on_dock, &msg_on_dock, NULL);
}

// ---------------------------------------------------------------------------
// Entity lifecycle — goto cleanup chain prevents fini of uninitialised handles
// ---------------------------------------------------------------------------
static bool create_entities(void)
{
    ctx.allocator = rcl_get_default_allocator();

    if (rclc_support_init(&ctx.support, 0, NULL, &ctx.allocator) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&ctx.node, "ina226_node", "", &ctx.support) != RCL_RET_OK)
        goto fail_support;

    if (rclc_publisher_init_best_effort(&ctx.pub_battery, &ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "battery_voltage") != RCL_RET_OK)
        goto fail_node;

    if (rclc_publisher_init_best_effort(&ctx.pub_charger, &ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "charger_voltage") != RCL_RET_OK)
        goto fail_pub_battery;

    // on_dock uses reliable QoS — a late subscriber needs current dock state.
    if (rclc_publisher_init_default(&ctx.pub_on_dock, &ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "on_dock") != RCL_RET_OK)
        goto fail_pub_charger;

    if (rclc_timer_init_default(&ctx.timer, &ctx.support,
            RCL_MS_TO_NS(TIMER_PERIOD_MS), timer_callback) != RCL_RET_OK)
        goto fail_pub_dock;

    ctx.executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&ctx.executor, &ctx.support.context, 1, &ctx.allocator) != RCL_RET_OK)
        goto fail_timer;

    if (rclc_executor_add_timer(&ctx.executor, &ctx.timer) != RCL_RET_OK)
        goto fail_executor;

    return true;

fail_executor:      rclc_executor_fini(&ctx.executor);
fail_timer:         rcl_timer_fini(&ctx.timer);
fail_pub_dock:      rcl_publisher_fini(&ctx.pub_on_dock, &ctx.node);
fail_pub_charger:   rcl_publisher_fini(&ctx.pub_charger, &ctx.node);
fail_pub_battery:   rcl_publisher_fini(&ctx.pub_battery, &ctx.node);
fail_node:          rcl_node_fini(&ctx.node);
fail_support:       rclc_support_fini(&ctx.support);
    return false;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&ctx.support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&ctx.pub_battery, &ctx.node);
    rcl_publisher_fini(&ctx.pub_charger, &ctx.node);
    rcl_publisher_fini(&ctx.pub_on_dock, &ctx.node);
    rcl_timer_fini(&ctx.timer);
    rclc_executor_fini(&ctx.executor);
    rcl_node_fini(&ctx.node);
    rclc_support_fini(&ctx.support);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(void)
{
    // micro-ROS transport (USB serial)
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Status LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // I2C1 — GPIO6=SDA1, GPIO7=SCL1
    i2c_init(i2c1, I2C_BAUD_RATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Hardware watchdog — auto-resets device if the loop stalls > 2 s
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    state = WAITING_AGENT;

    while (true) {
        watchdog_update();
        set_state_led(state);

        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(AGENT_POLL_MS,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(
                                 AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                        ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;

            case AGENT_AVAILABLE:
                if (create_entities()) {
                    rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
                    state = AGENT_CONNECTED;
                } else {
                    // create_entities already cleaned up via goto chain
                    state = WAITING_AGENT;
                }
                break;

            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(AGENT_KEEPALIVE_MS,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(
                                 AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                        ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&ctx.executor, RCL_MS_TO_NS(100));
                }
                break;

            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;

            default:
                break;
        }
    }
    // NOTREACHED
}
