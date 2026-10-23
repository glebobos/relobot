#include "serial_rx.h"

#include "zenoh-pico/protocol/codec/serial.h"

void serial_rx_reset(serial_rx_t *receiver)
{
    receiver->used = 0;
    receiver->discarding = false;
}

static bool valid_cobs(const uint8_t *encoded, size_t length)
{
    if (length < 2 || encoded[length - 1] != 0) {
        return false;
    }
    size_t offset = 0;
    size_t end = length - 1;
    while (offset < end) {
        uint8_t code = encoded[offset++];
        if (code == 0 || (size_t)(code - 1) > end - offset) {
            return false;
        }
        offset += code - 1;
    }
    return offset == end;
}

serial_rx_result_t serial_rx_poll(serial_rx_t *receiver, serial_rx_getc_t get_byte,
                                  void *context, uint8_t *payload, size_t capacity,
                                  size_t *received)
{
    *received = 0;
    for (size_t consumed = 0; consumed < sizeof(receiver->encoded); ++consumed) {
        int byte = get_byte(context);
        if (byte == SERIAL_RX_EMPTY) {
            return SERIAL_RX_PENDING;
        }
        if (byte < 0 || byte > UINT8_MAX) {
            serial_rx_reset(receiver);
            return SERIAL_RX_DISCONNECTED;
        }
        if (receiver->discarding) {
            if (byte == 0) {
                serial_rx_reset(receiver);
                return SERIAL_RX_INVALID;
            }
            continue;
        }
        if (receiver->used == sizeof(receiver->encoded)) {
            receiver->used = 0;
            receiver->discarding = byte != 0;
            if (byte == 0) {
                return SERIAL_RX_INVALID;
            }
            continue;
        }
        receiver->encoded[receiver->used++] = (uint8_t)byte;
        if (byte != 0) {
            continue;
        }
        size_t frame_size = receiver->used;
        receiver->used = 0;
        if (!valid_cobs(receiver->encoded, frame_size)) {
            return SERIAL_RX_INVALID;
        }
        uint8_t header = 0;
        size_t decoded_size = _z_serial_msg_deserialize(
            receiver->encoded, frame_size, payload, capacity, &header,
            receiver->decoded, sizeof(receiver->decoded));
        if (decoded_size == SIZE_MAX || decoded_size > _Z_SERIAL_MTU_SIZE) {
            return SERIAL_RX_INVALID;
        }
        if (header != 0) {
            return SERIAL_RX_CONTROL;
        }
        if (decoded_size == 0) {
            return SERIAL_RX_INVALID;
        }
        *received = decoded_size;
        return SERIAL_RX_FRAME;
    }
    return SERIAL_RX_PENDING;
}
