#ifndef RELOBOT_BOARD_H
#define RELOBOT_BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void relobot_board_init(void);
void relobot_board_led_set(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif

#endif
