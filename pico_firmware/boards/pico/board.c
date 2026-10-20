#include "relobot/board.h"

#include "board_pins.h"
#include "pico/stdlib.h"

void relobot_board_init(void)
{
    stdio_init_all();
    gpio_init(RELOBOT_STATUS_LED_PIN);
    gpio_set_dir(RELOBOT_STATUS_LED_PIN, GPIO_OUT);
    gpio_put(RELOBOT_STATUS_LED_PIN, 0);
}

void relobot_board_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    gpio_put(RELOBOT_STATUS_LED_PIN, (green > 0 || red > 0 || blue > 0));
}
