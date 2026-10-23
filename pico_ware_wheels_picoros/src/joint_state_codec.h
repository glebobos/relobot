#ifndef JOINT_STATE_CODEC_H
#define JOINT_STATE_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sensor_msgs/msg/joint_state.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JOINT_STATE_CDR_MAX_SIZE 512

bool joint_state_codec_init(void);
const char *joint_state_codec_hash(void);

bool joint_state_codec_encode(const sensor_msgs__msg__JointState *msg,
                              uint8_t *payload,
                              size_t max_len,
                              size_t *actual_length);

bool joint_state_codec_decode(const uint8_t *payload,
                              size_t length,
                              sensor_msgs__msg__JointState *msg);

void joint_state_msg_init_state(sensor_msgs__msg__JointState *msg,
                                rosidl_runtime_c__String *name_arr,
                                char *name0,
                                char *name1,
                                double *pos_arr,
                                double *vel_arr,
                                double *eff_arr,
                                char *frame_id_buf,
                                size_t frame_id_cap);

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
                              size_t frame_id_cap);

#ifdef __cplusplus
}
#endif

#endif /* JOINT_STATE_CODEC_H */
