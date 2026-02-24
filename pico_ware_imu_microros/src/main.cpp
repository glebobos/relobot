#include <stdio.h>
#include <math.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico_uart_transports.h"
#include "ICM20948/Arduino-ICM20948.h"

// RP2040 i2c1: GPIO22 = SDA, GPIO23 = SCL
#define I2C_SDA_PIN  22
#define I2C_SCL_PIN  23
#define ICM_ADDR     0x69

// Global defined in Arduino-ICM20948.cpp — set before icm20948.init()
extern uint8_t I2C_Address;

static ArduinoICM20948 icm20948;

static rcl_publisher_t publisher;
static sensor_msgs__msg__Imu imu_msg;
static rcl_timer_t timer;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;

static char frame_id_buf[] = "imu_link";

#define EXECUTE_EVERY_N_MS(MS, X) do {                        \
    static volatile int64_t init = -1;                        \
    if (init == -1) { init = uxr_millis(); }                  \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    // icm20948.task() is called in the main loop — just read current data here

    if (icm20948.gyroDataIsReady()) {
        float gx, gy, gz;
        icm20948.readGyroData(&gx, &gy, &gz);
        // dps → rad/s
        const float DPS_TO_RADS = 3.14159265f / 180.0f;
        imu_msg.angular_velocity.x = gx * DPS_TO_RADS;
        imu_msg.angular_velocity.y = gy * DPS_TO_RADS;
        imu_msg.angular_velocity.z = gz * DPS_TO_RADS;
    }

    if (icm20948.linearAccelDataIsReady()) {
        float ax, ay, az;
        icm20948.readLinearAccelData(&ax, &ay, &az);
        // g → m/s²
        const float G = 9.80665f;
        imu_msg.linear_acceleration.x = ax * G;
        imu_msg.linear_acceleration.y = ay * G;
        imu_msg.linear_acceleration.z = az * G;
    }

    if (icm20948.quat6DataIsReady()) {
        float qw, qx, qy, qz;
        icm20948.readQuat6Data(&qw, &qx, &qy, &qz);
        imu_msg.orientation.w = qw;
        imu_msg.orientation.x = qx;
        imu_msg.orientation.y = qy;
        imu_msg.orientation.z = qz;
    }

    uint64_t now_us = time_us_64();
    imu_msg.header.stamp.sec     = (int32_t)(now_us / 1000000ULL);
    imu_msg.header.stamp.nanosec = (uint32_t)((now_us % 1000000ULL) * 1000ULL);

    rcl_publish(&publisher, &imu_msg, NULL);
}

static bool create_entities(void)
{
    allocator = rcl_get_default_allocator();

    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_node_init_default(&node, "imu_node", "", &support);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu");
    if (rc != RCL_RET_OK) return false;

    // 30 Hz ≈ 33 ms
    rc = rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(33), timer_callback);
    if (rc != RCL_RET_OK) return false;

    executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_executor_add_timer(&executor, &timer);
    if (rc != RCL_RET_OK) return false;

    return true;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

static void init_imu_msg(void)
{
    memset(&imu_msg, 0, sizeof(imu_msg));

    imu_msg.header.frame_id.data     = frame_id_buf;
    imu_msg.header.frame_id.size     = strlen(frame_id_buf);
    imu_msg.header.frame_id.capacity = sizeof(frame_id_buf);

    // Orientation covariance — diagonal 0.001
    imu_msg.orientation_covariance[0] = 0.001;
    imu_msg.orientation_covariance[4] = 0.001;
    imu_msg.orientation_covariance[8] = 0.001;

    // Angular velocity covariance — diagonal 0.0001
    imu_msg.angular_velocity_covariance[0] = 0.0001;
    imu_msg.angular_velocity_covariance[4] = 0.0001;
    imu_msg.angular_velocity_covariance[8] = 0.0001;

    // Linear acceleration covariance — diagonal 0.01
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
}

int main(void)
{
    // micro-ROS transport init (USB serial)
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // I2C init — i2c1, GPIO22=SDA, GPIO23=SCL, 400 kHz
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Override default 0x68 to 0x69 before init
    I2C_Address = ICM_ADDR;

    ArduinoICM20948Settings icmSettings = {};
    icmSettings.i2c_speed                    = 400000;
    icmSettings.is_SPI                       = false;
    icmSettings.enable_gyroscope             = false;
    icmSettings.enable_accelerometer         = false;
    icmSettings.enable_magnetometer          = false;
    icmSettings.enable_gravity               = false;
    icmSettings.enable_linearAcceleration    = false;
    icmSettings.enable_quaternion6           = true;
    icmSettings.enable_quaternion9           = false;
    icmSettings.enable_har                   = false;
    icmSettings.enable_steps                 = false;
    icmSettings.gyroscope_frequency          = 30;
    icmSettings.linearAcceleration_frequency = 30;
    icmSettings.quaternion6_frequency        = 30;

    icm20948.init(icmSettings);

    init_imu_msg();

    state = WAITING_AGENT;

    while (true) {
        // Always pump DMP FIFO regardless of agent state
        icm20948.task();

        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) destroy_entities();
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(1000,
                    state = (RCL_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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

    return 0;
}
