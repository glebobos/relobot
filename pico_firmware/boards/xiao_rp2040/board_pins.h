#ifndef RELOBOT_XIAO_RP2040_BOARD_PINS_H
#define RELOBOT_XIAO_RP2040_BOARD_PINS_H

/* Discrete user RGB LED channels (Active-Low: 0 = ON, 1 = OFF) */
#define XIAO_LED_RED_PIN        17
#define XIAO_LED_GREEN_PIN      16
#define XIAO_LED_BLUE_PIN       25

#define RELOBOT_STATUS_LED_PIN  XIAO_NEOPIXEL_PIN

/* Onboard WS2812 (NeoPixel) */
#define XIAO_NEOPIXEL_PIN       12
#define XIAO_NEOPIXEL_PWR_PIN   11

/* XIAO header pin to RP2040 GPIO mapping (D0 - D10) */
#define XIAO_D0_PIN             26  /* ADC0 */
#define XIAO_D1_PIN             27  /* ADC1 */
#define XIAO_D2_PIN             28  /* ADC2 */
#define XIAO_D3_PIN             29  /* ADC3 */
#define XIAO_D4_PIN             6   /* I2C SDA */
#define XIAO_D5_PIN             7   /* I2C SCL */
#define XIAO_D6_PIN             0   /* UART TX */
#define XIAO_D7_PIN             1   /* UART RX */
#define XIAO_D8_PIN             2   /* SPI SCK */
#define XIAO_D9_PIN             4   /* SPI MISO */
#define XIAO_D10_PIN            3   /* SPI MOSI */

#endif

