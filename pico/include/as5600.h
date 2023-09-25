#ifndef _AS5600_H_
#define _AS5600_H_
#include <stdint.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

#define AS5600_ADDR 0x36
#define AS5600_ANGLE_REG 0x0E

/**
 * Initialize the I2C bus and pins where the AS5600 is connected.
 * Do not use if the I2C bus has already been initialized with i2c_init().
 * 
 * @param i2c I2C bus being used(i2c0, i2c1)
 * @param gpio_sda SDA / Data gpio pin number
 * @param gpio_scl SCL / Clock gpio pin number
*/
void as5600_i2c_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl);

/**
 * Read the current motor angle from the sensor. Data is a 12-bit integer and maps
 * 0->4095, wrapping back to 0 at one full revolution.
 * 
 * @param i2c I2C bus the device is conected to
 * @return Current angle reading as a 12-bit int
*/
uint16_t as5600_read_angle(i2c_inst_t *i2c);

#endif