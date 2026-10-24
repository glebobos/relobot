#include "joint_state_codec.h"

#include <stdio.h>
#include <string.h>

#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/detail/joint_state__functions.h>
#include <sensor_msgs/msg/detail/joint_state__rosidl_typesupport_microxrcedds_c.h>
#include <ucdr/microcdr.h>

static const message_type_support_callbacks_t *callbacks;
/* ROS 2 Lyrical REP-2011 RIHS hash for sensor_msgs/msg/JointState */
static const char *type_hash = "a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e";

bool joint_state_codec_init(void)
{
    const rosidl_message_type_support_t *support =
        ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
            rosidl_typesupport_microxrcedds_c, sensor_msgs, msg, JointState)();
    if (support == NULL || support->data == NULL) {
        return false;
    }
    callbacks = support->data;
    if (callbacks->cdr_serialize == NULL || callbacks->cdr_deserialize == NULL) {
        return false;
    }
    return true;
}

const char *joint_state_codec_hash(void)
{
    return type_hash;
}

bool joint_state_codec_encode(const sensor_msgs__msg__JointState *msg,
                              uint8_t *payload,
                              size_t max_len,
                              size_t *actual_length)
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

bool joint_state_codec_decode(const uint8_t *payload,
                              size_t length,
                              sensor_msgs__msg__JointState *msg)
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

void joint_state_msg_init_state(sensor_msgs__msg__JointState *msg,
                                rosidl_runtime_c__String *name_arr,
                                char *name0,
                                char *name1,
                                double *pos_arr,
                                double *vel_arr,
                                double *eff_arr,
                                char *frame_id_buf,
                                size_t frame_id_cap)
{
    if (!msg) return;
    memset(msg, 0, sizeof(*msg));

    if (name_arr && name0 && name1) {
        name_arr[0].data = name0;
        name_arr[0].size = strlen(name0);
        name_arr[0].capacity = strlen(name0) + 1;

        name_arr[1].data = name1;
        name_arr[1].size = strlen(name1);
        name_arr[1].capacity = strlen(name1) + 1;

        msg->name.data = name_arr;
        msg->name.size = 2;
        msg->name.capacity = 2;
    }

    if (pos_arr) {
        msg->position.data = pos_arr;
        msg->position.size = 2;
        msg->position.capacity = 2;
    }

    if (vel_arr) {
        msg->velocity.data = vel_arr;
        msg->velocity.size = 2;
        msg->velocity.capacity = 2;
    }

    if (eff_arr) {
        msg->effort.data = eff_arr;
        msg->effort.size = 2;
        msg->effort.capacity = 2;
    }

    if (frame_id_buf) {
        msg->header.frame_id.data = frame_id_buf;
        msg->header.frame_id.size = strlen(frame_id_buf);
        msg->header.frame_id.capacity = frame_id_cap;
    }
}

void joint_state_msg_init_cmd(sensor_msgs__msg__JointState *msg,
                              rosidl_runtime_c__String *name_arr,
                              char *name0_buf,
                              size_t name0_cap,
                              char *name1_buf,
                              size_t name1_cap,
                              double *pos_arr,
                              double *vel_arr,
                              double *eff_arr,
                              char *frame_id_buf,
                              size_t frame_id_cap)
{
    if (!msg) return;
    memset(msg, 0, sizeof(*msg));

    if (name_arr && name0_buf && name1_buf) {
        name0_buf[0] = '\0';
        name1_buf[0] = '\0';
        name_arr[0].data = name0_buf;
        name_arr[0].size = 0;
        name_arr[0].capacity = name0_cap;

        name_arr[1].data = name1_buf;
        name_arr[1].size = 0;
        name_arr[1].capacity = name1_cap;

        msg->name.data = name_arr;
        msg->name.size = 0;
        msg->name.capacity = 2;
    }

    if (pos_arr) {
        msg->position.data = pos_arr;
        msg->position.size = 0;
        msg->position.capacity = 2;
    }

    if (vel_arr) {
        msg->velocity.data = vel_arr;
        msg->velocity.size = 0;
        msg->velocity.capacity = 2;
    }

    if (eff_arr) {
        msg->effort.data = eff_arr;
        msg->effort.size = 0;
        msg->effort.capacity = 2;
    }

    if (frame_id_buf) {
        frame_id_buf[0] = '\0';
        msg->header.frame_id.data = frame_id_buf;
        msg->header.frame_id.size = 0;
        msg->header.frame_id.capacity = frame_id_cap;
    }
}
