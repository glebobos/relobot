#include "relobot/board.h"

#include "board_pins.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "xiao_ws2812.pio.h"

static uint led_state_machine;

void relobot_board_init(void)
{
    stdio_init_all();

    /* Enable NeoPixel power */
    gpio_init(XIAO_NEOPIXEL_PWR_PIN);
    gpio_set_dir(XIAO_NEOPIXEL_PWR_PIN, GPIO_OUT);
    gpio_put(XIAO_NEOPIXEL_PWR_PIN, 1);

    /* Initialize PIO for WS2812 on GPIO 22 */
    PIO led_pio = pio0;
    led_state_machine = pio_claim_unused_sm(led_pio, true);
    uint program_offset = pio_add_program(led_pio, &xiao_ws2812_program);
    pio_sm_config config = xiao_ws2812_program_get_default_config(program_offset);

    sm_config_set_sideset_pins(&config, XIAO_NEOPIXEL_PIN);
    sm_config_set_out_shift(&config, false, true, 24);
    sm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / (800000.0f * 12.0f));
    pio_gpio_init(led_pio, XIAO_NEOPIXEL_PIN);
    pio_sm_set_consecutive_pindirs(led_pio, led_state_machine, XIAO_NEOPIXEL_PIN, 1, true);
    pio_sm_init(led_pio, led_state_machine, program_offset, &config);
    pio_sm_set_enabled(led_pio, led_state_machine, true);

    relobot_board_led_set(0, 0, 0);
}

void relobot_board_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    /* WS2812 uses GRB ordering */
    uint32_t grb = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
    pio_sm_put_blocking(pio0, led_state_machine, grb << 8);
    busy_wait_at_least_cycles((clock_get_hz(clk_sys) / 1000000u) * 400u);
}
