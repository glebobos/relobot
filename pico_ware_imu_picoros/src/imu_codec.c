#include "imu_codec.h"

#include <stdio.h>
#include <string.h>

#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/detail/imu__functions.h>
#include <sensor_msgs/msg/detail/imu__rosidl_typesupport_microxrcedds_c.h>
#include <ucdr/microcdr.h>

static const message_type_support_callbacks_t *callbacks;
/* ROS 2 Lyrical REP-2011 RIHS hash for sensor_msgs/msg/Imu */
static const char *type_hash = "7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";

bool imu_codec_init(void)
{
    const rosidl_message_type_support_t *support =
        ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
            rosidl_typesupport_microxrcedds_c, sensor_msgs, msg, Imu)();
    if (support == NULL || support->data == NULL) {
        return false;
    }
    callbacks = support->data;
    if (callbacks->cdr_serialize == NULL || callbacks->cdr_deserialize == NULL) {
        return false;
    }
    return true;
}

const char *imu_codec_hash(void)
{
    return type_hash;
}

bool imu_codec_encode(const sensor_msgs__msg__Imu *msg, uint8_t *payload, size_t max_len, size_t *actual_length)
{
    if (callbacks == NULL || msg == NULL || payload == NULL || max_len < 4) {
        return false;
    }
    memcpy(payload, (uint8_t[]){0, 1, 0, 0}, 4);
    ucdrBuffer cdr;
    ucdr_init_buffer_origin_offset_endian(&cdr, payload + 4, (uint32_t)(max_len - 4),
                                          0, 0, UCDR_LITTLE_ENDIANNESS);
    if (!callbacks->cdr_serialize(msg, &cdr)) {
        return false;
    }
    size_t len = 4 + ucdr_buffer_length(&cdr);
    if (actual_length != NULL) {
        *actual_length = len;
    }
    return true;
}

bool imu_codec_decode(const uint8_t *payload, size_t length, sensor_msgs__msg__Imu *msg)
{
    if (callbacks == NULL || payload == NULL || msg == NULL ||
        length < 5 || payload[0] != 0 || payload[1] > 1 ||
        payload[2] != 0 || payload[3] != 0) {
        return false;
    }
    ucdrBuffer cdr;
    ucdr_init_buffer_origin_offset_endian(&cdr, (uint8_t *)(payload + 4), (uint32_t)(length - 4), 0, 0,
        payload[1] == 1 ? UCDR_LITTLE_ENDIANNESS : UCDR_BIG_ENDIANNESS);
    if (!callbacks->cdr_deserialize(&cdr, msg)) {
        return false;
    }
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

    /* Orientation covariance — diagonal 0.001 */
    msg->orientation_covariance[0] = 0.001;
    msg->orientation_covariance[4] = 0.001;
    msg->orientation_covariance[8] = 0.001;

    /* Angular velocity covariance — diagonal 0.0001 */
    msg->angular_velocity_covariance[0] = 0.0001;
    msg->angular_velocity_covariance[4] = 0.0001;
    msg->angular_velocity_covariance[8] = 0.0001;

    /* Linear acceleration covariance — diagonal 0.01 */
    msg->linear_acceleration_covariance[0] = 0.01;
    msg->linear_acceleration_covariance[4] = 0.01;
    msg->linear_acceleration_covariance[8] = 0.01;

    /* Default orientation: identity quaternion */
    msg->orientation.w = 1.0;
}
