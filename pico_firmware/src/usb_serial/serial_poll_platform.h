#ifndef RELOBOT_SERIAL_POLL_PLATFORM_H
#define RELOBOT_SERIAL_POLL_PLATFORM_H

#include "serial_rx.h"

bool serial_poll_is_usb(_z_sys_net_socket_t socket);
int serial_poll_get_byte(void *context);

#endif
