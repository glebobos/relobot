#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_I2C_SDA_PIN     6
#define IMU_I2C_SCL_PIN     7
#define IMU_I2C_ADDR        0x68
#define IMU_I2C_BAUDRATE    400000

typedef struct {
    float orientation[4];        /* qw, qx, qy, qz */
    float angular_velocity[3];   /* gx, gy, gz in rad/s */
    float linear_acceleration[3];/* ax, ay, az in m/s^2 */
    bool orientation_valid;
    bool angular_velocity_valid;
    bool linear_acceleration_valid;
} imu_sensor_data_t;

void imu_sensor_hardware_init(void);
void imu_sensor_task(void);
bool imu_sensor_read_data(imu_sensor_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SENSOR_H */
