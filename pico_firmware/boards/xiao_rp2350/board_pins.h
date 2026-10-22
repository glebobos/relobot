#ifndef RELOBOT_XIAO_RP2350_BOARD_PINS_H
#define RELOBOT_XIAO_RP2350_BOARD_PINS_H

/* Onboard WS2812 (NeoPixel). Unlike XIAO RP2040, this board has no separate
 * discrete R/G/B LEDs (confirmed via Seeed's XIAO RP2350 pin map and the
 * pico-sdk seeed_xiao_rp2350.h board header: PICO_DEFAULT_WS2812_PIN=22,
 * PICO_DEFAULT_WS2812_POWER_PIN=23). */
#define RELOBOT_STATUS_LED_PIN  XIAO_NEOPIXEL_PIN

#define XIAO_NEOPIXEL_PIN       22
#define XIAO_NEOPIXEL_PWR_PIN   23

#endif
