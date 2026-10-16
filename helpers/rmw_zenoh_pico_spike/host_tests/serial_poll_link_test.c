#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "serial_poll_platform.h"
#include "zenoh-pico/link/manager.h"
#include "zenoh-pico/link/transport/serial.h"
#include "zenoh-pico/protocol/codec/serial.h"
#include "zenoh-pico/transport/unicast/read.h"
#include "zenoh-pico/transport/unicast/transport.h"

static uint8_t encoded[_Z_SERIAL_MAX_COBS_BUF_SIZE];
static size_t available;
static size_t offset;
static unsigned int blocking_reads;
static bool usb_socket = true;

z_result_t __real__z_new_link_serial(_z_link_t *link, _z_endpoint_t endpoint);

bool serial_poll_is_usb(_z_sys_net_socket_t socket)
{
    (void)socket;
    return usb_socket;
}

int serial_poll_get_byte(void *context)
{
    (void)context;
    return offset < available ? encoded[offset++] : SERIAL_RX_EMPTY;
}

size_t __wrap__z_read_serial(_z_sys_net_socket_t socket, uint8_t *payload, size_t capacity)
{
    (void)socket;
    assert(capacity > 0);
    ++blocking_reads;
    payload[0] = 55;
    return 1;
}

int main(void)
{
    _z_link_t reference = {0};
    _z_link_t link = {0};
    _z_endpoint_t endpoint = {0};
    assert(__real__z_new_link_serial(&reference, endpoint) == _Z_RES_OK);
    assert(_z_new_link_serial(&link, endpoint) == _Z_RES_OK);
    assert(link._read_socket_f != reference._read_socket_f);
    assert(link._read_f == reference._read_f);
    assert(link._read_exact_f == reference._read_exact_f);
    assert(link._write_f == reference._write_f);
    assert(link._close_f == reference._close_f);
    const uint8_t payload[] = {2, 0, 42};
    uint8_t workspace[_Z_SERIAL_MAX_COBS_BUF_SIZE];
    size_t frame_size = _z_serial_msg_serialize(encoded, sizeof(encoded), payload,
        sizeof(payload), 0, workspace, sizeof(workspace));
    assert(frame_size != SIZE_MAX);
    _z_sys_net_socket_t socket = {._fd = -1};
    uint8_t output[16];
    available = frame_size / 2;
    assert(link._read_socket_f(socket, output, sizeof(output)) == SIZE_MAX);
    available = frame_size;
    assert(link._read_socket_f(socket, output, sizeof(output)) == sizeof(payload));
    assert(memcmp(output, payload, sizeof(payload)) == 0 && blocking_reads == 0);
    offset = 0;
    available = frame_size / 2;
    assert(link._read_socket_f(socket, output, sizeof(output)) == SIZE_MAX);
    _z_serial_close(&socket);
    offset = 0;
    available = frame_size;
    assert(link._read_socket_f(socket, output, sizeof(output)) == sizeof(payload));
    assert(memcmp(output, payload, sizeof(payload)) == 0);
    usb_socket = false;
    assert(link._read_socket_f(socket, output, sizeof(output)) == 1);
    assert(output[0] == 55 && blocking_reads == 1);
    _z_transport_unicast_t transport = {0};
    _z_executor_t executor = _z_executor_new();
    transport._common._state = _Z_TRANSPORT_STATE_CLOSED;
    assert(_zp_unicast_read_task_fn(&transport, &executor)._status == _Z_FUT_STATUS_READY);
    transport._common._state = _Z_TRANSPORT_STATE_RECONNECTING;
    assert(_zp_unicast_read_task_fn(&transport, &executor)._status == _Z_FUT_STATUS_SUSPENDED);
    _z_executor_destroy(&executor);
    puts("PASS: actual link factory installs async RX, preserves sync hooks, resets on close and retains future stop states.");
    return 0;
}
