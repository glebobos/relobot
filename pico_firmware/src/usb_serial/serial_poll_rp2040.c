#include "serial_poll_platform.h"

int _z_usb_uart_try_getc(void);

bool serial_poll_is_usb(_z_sys_net_socket_t socket)
{
    return socket._serial == NULL;
}

int serial_poll_get_byte(void *context)
{
    (void)context;
    return _z_usb_uart_try_getc();
}
