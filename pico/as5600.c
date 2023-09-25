#include "as5600.h"
#include "pico/stdlib.h"

/**
 * Initialize the I2C bus and pins where the AS5600 is connected.
 * Do not use if the I2C bus has already been initialized with i2c_init().
 * 
 * @param i2c I2C bus being used(i2c0, i2c1)
 * @param gpio_sda SDA / Data gpio pin number
 * @param gpio_scl SCL / Clock gpio pin number
*/
void as5600_i2c_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl)
{
    i2c_init(i2c, 400*1000);
    gpio_set_function(gpio_sda, GPIO_FUNC_I2C);
    gpio_set_function(gpio_scl, GPIO_FUNC_I2C);
    gpio_pull_up(gpio_sda);
    gpio_pull_up(gpio_scl);
}

/**
 * Read the current motor angle from the sensor. Data is a 12-bit integer and maps
 * 0->4095, wrapping back to 0 at one full revolution.
 * 
 * @param i2c I2C bus the device is conected to
 * @return Current angle reading as a 12-bit int
*/
uint16_t as5600_read_angle(i2c_inst_t *i2c)
{
    uint8_t recv[2] = {0,0};
    uint8_t reg = AS5600_ANGLE_REG;
    i2c_write_blocking(i2c, AS5600_ADDR, &reg, 1, false);
    i2c_read_blocking(i2c, AS5600_ADDR, recv, 2, false);

    // Bit shift to combine the high [0] byte and low [1] byte
    int out = (recv[0] << 8) | recv[1];
    
    // 12-bit int, make sure first 4 bits are ignored
    out = out & 0xFFF;

    return out;
}