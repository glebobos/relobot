#ifndef PICOROS_INA226_H
#define PICOROS_INA226_H

#include <stdbool.h>
#include <stdint.h>

#define INA226_I2C_SDA_PIN          6
#define INA226_I2C_SCL_PIN          7
#define INA226_I2C_BAUD_RATE        400000
#define INA226_I2C_TIMEOUT_MS       5

#define INA226_ADDR_BATTERY         0x40    /* Battery pack */
#define INA226_ADDR_CHARGER         0x41    /* Docking station charger */
#define INA226_REG_BUS_VOLTAGE      0x02

#define INA226_VOLTAGE_LSB          0.00125f /* 1.25 mV per LSB */
#define DOCK_VOLTAGE_THRESHOLD      12.5f    /* Voltage threshold for on-dock state */

/* Convert raw 16-bit bus voltage register value to Volts */
float ina226_raw_to_voltage(uint16_t raw);

/* Check if charger voltage indicates robot is docked (> threshold, guarded against NAN) */
bool ina226_is_on_dock(float v_charger);

/* Initialize I2C hardware pins and peripheral */
void ina226_hardware_init(void);

/* Read bus voltage register from INA226 at given I2C address. Returns NAN on I2C error/timeout. */
float ina226_read_bus_voltage(uint8_t addr);

#endif /* PICOROS_INA226_H */
