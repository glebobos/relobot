#include "ina226.h"

#include <math.h>

#if !RELOBOT_INA226_BENCH
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#endif

float ina226_raw_to_voltage(uint16_t raw)
{
    return (float)raw * INA226_VOLTAGE_LSB;
}

bool ina226_is_on_dock(float v_charger)
{
    return !isnan(v_charger) && (v_charger > DOCK_VOLTAGE_THRESHOLD);
}

void ina226_hardware_init(void)
{
#if !RELOBOT_INA226_BENCH
    i2c_init(i2c1, INA226_I2C_BAUD_RATE);
    gpio_set_function(INA226_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(INA226_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(INA226_I2C_SDA_PIN);
    gpio_pull_up(INA226_I2C_SCL_PIN);
#endif
}

float ina226_read_bus_voltage(uint8_t addr)
{
#if RELOBOT_INA226_BENCH
    /* In bench mode without physical sensors: return mock values */
    if (addr == INA226_ADDR_BATTERY) {
        return 14.8f; /* Nominal battery pack voltage */
    } else if (addr == INA226_ADDR_CHARGER) {
        return 0.0f;  /* Off dock */
    }
    return NAN;
#else
    uint8_t reg = INA226_REG_BUS_VOLTAGE;
    int wrc = i2c_write_blocking_until(i2c1, addr, &reg, 1, true,
                  make_timeout_time_ms(INA226_I2C_TIMEOUT_MS));
    if (wrc < 0) {
        return NAN;
    }

    uint8_t buf[2] = {0, 0};
    int rrc = i2c_read_blocking_until(i2c1, addr, buf, 2, false,
                  make_timeout_time_ms(INA226_I2C_TIMEOUT_MS));
    if (rrc < 0) {
        return NAN;
    }

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return ina226_raw_to_voltage(raw);
#endif
}
