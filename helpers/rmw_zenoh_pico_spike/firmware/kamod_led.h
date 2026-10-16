#ifndef KAMOD_LED_H
#define KAMOD_LED_H

#include <stdint.h>

void kamod_led_init(void);
void kamod_led_set(uint8_t red, uint8_t green, uint8_t blue);

#endif
