#ifndef PICOROS_BOOL_CODEC_H
#define PICOROS_BOOL_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BOOL_CDR_SIZE 8u

bool bool_codec_init(void);
const char *bool_codec_hash(void);
bool bool_codec_encode(bool value, uint8_t payload[BOOL_CDR_SIZE], size_t *actual_length);
bool bool_codec_decode(const uint8_t *payload, size_t length, bool *value);

#endif
