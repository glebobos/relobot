#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "serial_rx.h"
#include "zenoh-pico/protocol/codec/serial.h"
#include "zenoh-pico/protocol/definitions/serial.h"

typedef struct {
    uint8_t bytes[_Z_SERIAL_MAX_COBS_BUF_SIZE * 4];
    size_t available;
    size_t offset;
    size_t calls;
    bool closed;
} input_t;

static int next_byte(void *context)
{
    input_t *input = context;
    ++input->calls;
    if (input->offset < input->available) {
        return input->bytes[input->offset++];
    }
    return input->closed ? SERIAL_RX_CLOSED : SERIAL_RX_EMPTY;
}

static int endless_bytes(void *context)
{
    input_t *input = context;
    ++input->calls;
    return 1;
}

static size_t encode(uint8_t *frame, const uint8_t *payload, size_t length, uint8_t header)
{
    uint8_t workspace[_Z_SERIAL_MAX_COBS_BUF_SIZE];
    size_t encoded = _z_serial_msg_serialize(frame, _Z_SERIAL_MAX_COBS_BUF_SIZE,
        payload, length, header, workspace, sizeof(workspace));
    assert(encoded != SIZE_MAX);
    return encoded;
}

static void test_fragmentation(const uint8_t *payload, size_t length)
{
    input_t input = {0};
    size_t frame_size = encode(input.bytes, payload, length, 0);
    uint8_t output[_Z_SERIAL_MTU_SIZE];
    for (size_t split = 0; split < frame_size; ++split) {
        serial_rx_t receiver = {0};
        size_t received = SIZE_MAX;
        input.available = split;
        input.offset = 0;
        assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                              &received) == SERIAL_RX_PENDING);
        assert(received == 0 && receiver.used == split);
        input.available = frame_size;
        assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                              &received) == SERIAL_RX_FRAME);
        assert(received == length && memcmp(output, payload, length) == 0);
        assert(receiver.used == 0);
    }
    serial_rx_t receiver = {0};
    input.offset = 0;
    for (size_t available = 1; available <= frame_size; ++available) {
        input.available = available;
        size_t received;
        serial_rx_result_t result = serial_rx_poll(&receiver, next_byte, &input,
            output, sizeof(output), &received);
        assert(result == (available == frame_size ? SERIAL_RX_FRAME : SERIAL_RX_PENDING));
    }
    assert(memcmp(output, payload, length) == 0);
}

static void test_reset_disconnect_and_queue(void)
{
    const uint8_t payload[] = {7, 0, 42, 255};
    input_t input = {0};
    size_t frame_size = encode(input.bytes, payload, sizeof(payload), 0);
    serial_rx_t receiver = {0};
    uint8_t output[16];
    size_t received;
    input.available = frame_size / 2;
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_PENDING);
    input.closed = true;
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_DISCONNECTED);
    assert(receiver.used == 0 && !receiver.discarding);
    input.closed = false;
    input.offset = 0;
    input.available = frame_size * 2;
    memcpy(input.bytes + frame_size, input.bytes, frame_size);
    for (unsigned int frame = 0; frame < 2; ++frame) {
        assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                              &received) == SERIAL_RX_FRAME);
        assert(received == sizeof(payload) && memcmp(output, payload, received) == 0);
    }
    input.offset = 0;
    input.available = 2;
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_PENDING);
    serial_rx_reset(&receiver);
    assert(receiver.used == 0 && !receiver.discarding);
}

static void test_invalid_and_control(void)
{
    const uint8_t payload[] = {4, 5, 6};
    input_t input = {0};
    serial_rx_t receiver = {0};
    uint8_t output[16];
    size_t received;
    input.available = encode(input.bytes, payload, sizeof(payload), _Z_FLAG_SERIAL_INIT | _Z_FLAG_SERIAL_ACK);
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_CONTROL);
    assert(received == 0);
    input.available = encode(input.bytes, payload, sizeof(payload), 0);
    input.offset = 0;
    assert(serial_rx_poll(&receiver, next_byte, &input, output, 1,
                          &received) == SERIAL_RX_INVALID);
    assert(received == 0);
    input.offset = 0;
    input.bytes[4] ^= 0x10;
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_INVALID);
    input.offset = 0;
    input.bytes[0] = 4;
    input.bytes[1] = 1;
    input.bytes[2] = 0;
    input.available = 3;
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_INVALID);
    input.offset = 0;
    input.bytes[0] = 0;
    input.available = 1;
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_INVALID);
    input.offset = 0;
    input.available = encode(input.bytes, payload, 0, 0);
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_INVALID);
}

static void test_overflow_and_work_budget(void)
{
    input_t input = {0};
    serial_rx_t receiver = {0};
    uint8_t output[16];
    size_t received;
    for (unsigned int poll = 0; poll < 2; ++poll) {
        input.calls = 0;
        assert(serial_rx_poll(&receiver, endless_bytes, &input, output, sizeof(output),
                              &received) == SERIAL_RX_PENDING);
        assert(input.calls == _Z_SERIAL_MAX_COBS_BUF_SIZE);
    }
    assert(receiver.discarding);
    serial_rx_reset(&receiver);
    const uint8_t payload[] = {99};
    memset(input.bytes, 1, _Z_SERIAL_MAX_COBS_BUF_SIZE + 4);
    input.bytes[_Z_SERIAL_MAX_COBS_BUF_SIZE + 4] = 0;
    size_t prefix = _Z_SERIAL_MAX_COBS_BUF_SIZE + 5;
    input.available = prefix + encode(input.bytes + prefix, payload, sizeof(payload), 0);
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_PENDING);
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_INVALID);
    assert(serial_rx_poll(&receiver, next_byte, &input, output, sizeof(output),
                          &received) == SERIAL_RX_FRAME);
    assert(received == sizeof(payload) && output[0] == payload[0]);
}

int main(void)
{
    const uint8_t payload[] = {0, 1, 0, 2, 255, 7};
    test_fragmentation(payload, sizeof(payload));
    uint8_t maximum[_Z_SERIAL_MTU_SIZE];
    memset(maximum, 0x7f, sizeof(maximum));
    test_fragmentation(maximum, sizeof(maximum));
    memset(maximum, 0, sizeof(maximum));
    test_fragmentation(maximum, sizeof(maximum));
    test_reset_disconnect_and_queue();
    test_invalid_and_control();
    test_overflow_and_work_budget();
    puts("PASS: fragmented pinned-codec frames, MTU, queue, disconnect/reset, control, corruption and bounded work.");
    return 0;
}
