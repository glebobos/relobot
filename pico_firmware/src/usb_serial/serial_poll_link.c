#include "serial_poll_platform.h"

#include "zenoh-pico/link/manager.h"
#include "zenoh-pico/runtime/executor.h"

static serial_rx_t usb_receiver;

z_result_t __real__z_new_link_serial(_z_link_t *link, _z_endpoint_t endpoint);
void __real__z_serial_close(_z_sys_net_socket_t *socket);
_z_fut_fn_result_t __real__zp_unicast_read_task_fn(void *transport, _z_executor_t *executor);

static size_t serial_poll_read_socket(_z_sys_net_socket_t socket, uint8_t *payload, size_t capacity)
{
    if (!serial_poll_is_usb(socket)) {
        return _z_read_serial(socket, payload, capacity);
    }
    size_t received = 0;
    serial_rx_result_t result = serial_rx_poll(&usb_receiver, serial_poll_get_byte, NULL,
                                               payload, capacity, &received);
    return result == SERIAL_RX_FRAME ? received : SIZE_MAX;
}

z_result_t __wrap__z_new_link_serial(_z_link_t *link, _z_endpoint_t endpoint)
{
    z_result_t result = __real__z_new_link_serial(link, endpoint);
    if (result == _Z_RES_OK) {
        link->_read_socket_f = serial_poll_read_socket;
    }
    return result;
}

void __wrap__z_serial_close(_z_sys_net_socket_t *socket)
{
    if (serial_poll_is_usb(*socket)) {
        serial_rx_reset(&usb_receiver);
    }
    __real__z_serial_close(socket);
}

_z_fut_fn_result_t __wrap__zp_unicast_read_task_fn(void *transport, _z_executor_t *executor)
{
    _z_fut_fn_result_t result = __real__zp_unicast_read_task_fn(transport, executor);
    if (result._status == _Z_FUT_STATUS_RUNNING) {
        return _z_fut_fn_result_wake_up_after(5);
    }
    return result;
}
