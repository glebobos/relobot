#include "imu_sensor.h"
#include <math.h>

#if !RELOBOT_IMU_BENCH
#if !defined(UNIT_TEST)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ICM20948/Arduino-ICM20948.h"

extern uint8_t I2C_Address;
static ArduinoICM20948 icm20948;
#endif
#endif

void imu_sensor_hardware_init(void)
{
#if !RELOBOT_IMU_BENCH && !defined(UNIT_TEST)
    i2c_init(i2c1, IMU_I2C_BAUDRATE);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    I2C_Address = IMU_I2C_ADDR;

    ArduinoICM20948Settings icmSettings = {};
    icmSettings.i2c_speed                    = IMU_I2C_BAUDRATE;
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
#endif
}

void imu_sensor_task(void)
{
#if !RELOBOT_IMU_BENCH && !defined(UNIT_TEST)
    icm20948.task();
#endif
}

bool imu_sensor_read_data(imu_sensor_data_t *data)
{
    if (data == NULL) {
        return false;
    }

#if RELOBOT_IMU_BENCH
    /* Mock bench telemetry: identity orientation, zero angular velocity, 1g vertical acceleration */
    data->orientation[0] = 1.0f; /* qw */
    data->orientation[1] = 0.0f; /* qx */
    data->orientation[2] = 0.0f; /* qy */
    data->orientation[3] = 0.0f; /* qz */
    data->orientation_valid = true;

    data->angular_velocity[0] = 0.0f;
    data->angular_velocity[1] = 0.0f;
    data->angular_velocity[2] = 0.0f;
    data->angular_velocity_valid = true;

    data->linear_acceleration[0] = 0.0f;
    data->linear_acceleration[1] = 0.0f;
    data->linear_acceleration[2] = 9.80665f; /* 1g */
    data->linear_acceleration_valid = true;

    return true;
#elif defined(UNIT_TEST)
    return false;
#else
    bool any_ready = false;
    data->orientation_valid = false;
    data->angular_velocity_valid = false;
    data->linear_acceleration_valid = false;

    if (icm20948.gyroDataIsReady()) {
        float gx, gy, gz;
        icm20948.readGyroData(&gx, &gy, &gz);
        const float DPS_TO_RADS = 3.14159265f / 180.0f;
        data->angular_velocity[0] = gx * DPS_TO_RADS;
        data->angular_velocity[1] = gy * DPS_TO_RADS;
        data->angular_velocity[2] = gz * DPS_TO_RADS;
        data->angular_velocity_valid = true;
        any_ready = true;
    }

    if (icm20948.linearAccelDataIsReady()) {
        float ax, ay, az;
        icm20948.readLinearAccelData(&ax, &ay, &az);
        const float G = 9.80665f;
        data->linear_acceleration[0] = ax * G;
        data->linear_acceleration[1] = ay * G;
        data->linear_acceleration[2] = az * G;
        data->linear_acceleration_valid = true;
        any_ready = true;
    }

    if (icm20948.quat6DataIsReady()) {
        float qw, qx, qy, qz;
        icm20948.readQuat6Data(&qw, &qx, &qy, &qz);
        data->orientation[0] = qw;
        data->orientation[1] = qx;
        data->orientation[2] = qy;
        data->orientation[3] = qz;
        data->orientation_valid = true;
        any_ready = true;
    }

    return any_ready;
#endif
}
