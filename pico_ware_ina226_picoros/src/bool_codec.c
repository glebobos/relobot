#include "bool_codec.h"

#include <stdio.h>
#include <string.h>

#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/detail/bool__functions.h>
#include <std_msgs/msg/detail/bool__rosidl_typesupport_microxrcedds_c.h>
#include <ucdr/microcdr.h>

static const message_type_support_callbacks_t *callbacks;
static char type_hash[2 * ROSIDL_TYPE_HASH_SIZE + 1];

bool bool_codec_init(void)
{
    const rosidl_type_hash_t *hash = std_msgs__msg__Bool__get_type_hash(NULL);
    const rosidl_message_type_support_t *support =
        ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
            rosidl_typesupport_microxrcedds_c, std_msgs, msg, Bool)();
    if (hash == NULL || hash->version != 1 || support == NULL || support->data == NULL) {
        return false;
    }
    callbacks = support->data;
    if (callbacks->cdr_serialize == NULL || callbacks->cdr_deserialize == NULL) {
        return false;
    }
    for (size_t index = 0; index < ROSIDL_TYPE_HASH_SIZE; ++index) {
        snprintf(type_hash + 2 * index, sizeof(type_hash) - 2 * index,
                 "%02x", hash->value[index]);
    }
    return true;
}

const char *bool_codec_hash(void)
{
    return type_hash;
}

bool bool_codec_encode(bool value, uint8_t payload[BOOL_CDR_SIZE], size_t *actual_length)
{
    if (callbacks == NULL || payload == NULL) {
        return false;
    }
    std_msgs__msg__Bool message = {.data = value};
    memcpy(payload, (uint8_t[]){0, 1, 0, 0}, 4);
    ucdrBuffer cdr;
    ucdr_init_buffer_origin_offset_endian(&cdr, payload + 4, BOOL_CDR_SIZE - 4,
                                         0, 0, UCDR_LITTLE_ENDIANNESS);
    if (!callbacks->cdr_serialize(&message, &cdr)) {
        return false;
    }
    size_t len = 4 + ucdr_buffer_length(&cdr);
    if (actual_length != NULL) {
        *actual_length = len;
    }
    return true;
}

bool bool_codec_decode(const uint8_t *payload, size_t length, bool *value)
{
    if (callbacks == NULL || payload == NULL || value == NULL ||
        length < 5 || payload[0] != 0 || payload[1] > 1 ||
        payload[2] != 0 || payload[3] != 0) {
        return false;
    }
    ucdrBuffer cdr;
    ucdr_init_buffer_origin_offset_endian(&cdr, (uint8_t *)(payload + 4), length - 4, 0, 0,
        payload[1] == 1 ? UCDR_LITTLE_ENDIANNESS : UCDR_BIG_ENDIANNESS);
    std_msgs__msg__Bool message = {0};
    if (!callbacks->cdr_deserialize(&cdr, &message)) {
        return false;
    }
    *value = message.data;
    return true;
}
