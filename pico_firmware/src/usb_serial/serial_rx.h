#ifndef RELOBOT_SERIAL_RX_H
#define RELOBOT_SERIAL_RX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "zenoh-pico/link/transport/serial_protocol.h"

enum { SERIAL_RX_CLOSED = -1, SERIAL_RX_EMPTY = -2 };

typedef enum {
    SERIAL_RX_PENDING,
    SERIAL_RX_FRAME,
    SERIAL_RX_DISCONNECTED,
    SERIAL_RX_INVALID,
    SERIAL_RX_CONTROL
} serial_rx_result_t;

typedef struct {
    uint8_t encoded[_Z_SERIAL_MAX_COBS_BUF_SIZE];
    uint8_t decoded[_Z_SERIAL_MAX_COBS_BUF_SIZE];
    size_t used;
    bool discarding;
} serial_rx_t;

typedef int (*serial_rx_getc_t)(void *context);

void serial_rx_reset(serial_rx_t *receiver);
serial_rx_result_t serial_rx_poll(serial_rx_t *receiver, serial_rx_getc_t get_byte,
                                  void *context, uint8_t *payload, size_t capacity,
                                  size_t *received);

#endif
