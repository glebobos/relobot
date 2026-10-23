#ifndef FLOAT32_CODEC_H
#define FLOAT32_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLOAT32_CDR_SIZE 8u

bool float32_codec_init(void);
const char *float32_codec_hash(void);
bool float32_codec_encode(float value, uint8_t payload[FLOAT32_CDR_SIZE]);
bool float32_codec_decode(uint8_t *payload, size_t length, float *value);

#ifdef __cplusplus
}
#endif

#endif /* FLOAT32_CODEC_H */
