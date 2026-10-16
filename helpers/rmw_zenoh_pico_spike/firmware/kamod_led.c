#include "kamod_led.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "kamod_ws2812.pio.h"

#define KAMOD_WS2812_PIN 16

static uint led_sm;

void kamod_led_init(void)
{
    PIO pio = pio0;
    led_sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &kamod_ws2812_program);
    pio_sm_config config = kamod_ws2812_program_get_default_config(offset);

    sm_config_set_sideset_pins(&config, KAMOD_WS2812_PIN);
    sm_config_set_out_shift(&config, false, true, 24);
    sm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / (800000.0f * 12.0f));
    pio_gpio_init(pio, KAMOD_WS2812_PIN);
    pio_sm_set_consecutive_pindirs(pio, led_sm, KAMOD_WS2812_PIN, 1, true);
    pio_sm_init(pio, led_sm, offset, &config);
    pio_sm_set_enabled(pio, led_sm, true);
}

void kamod_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    uint32_t grb = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
    pio_sm_put_blocking(pio0, led_sm, grb << 8);
    busy_wait_at_least_cycles((clock_get_hz(clk_sys) / 1000000u) * 400u);
}
