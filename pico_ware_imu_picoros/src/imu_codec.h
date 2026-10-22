#ifndef IMU_CODEC_H
#define IMU_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sensor_msgs/msg/imu.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CDR_MAX_SIZE 512

bool imu_codec_init(void);
const char *imu_codec_hash(void);
bool imu_codec_encode(const sensor_msgs__msg__Imu *msg, uint8_t *payload, size_t max_len, size_t *actual_length);
bool imu_codec_decode(const uint8_t *payload, size_t length, sensor_msgs__msg__Imu *msg);
void imu_msg_init_defaults(sensor_msgs__msg__Imu *msg, char *frame_id_buf, size_t frame_id_buf_size);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CODEC_H */
