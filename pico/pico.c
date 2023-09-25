#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include "motor.h"
#include "as5600.h"

// Encoder Config TODO CHANGE ME
#define L_ENC_I2C i2c0
#define L_ENC_SDA 1
#define L_ENC_SCL 2

#define R_ENC_I2C i2c1
#define R_ENC_SDA 3
#define R_ENC_SCL 4

// PWM Config
#define M1_PWM1 5
#define M1_PWM2 6

#define M2_PWM1 7
#define M2_PWM2 8

#define M3_PWM1 9
#define M3_PWM2 10

#define M4_PWM1 11
#define M4_PWM2 12


int main()
{
    stdio_init_all();

    
    

    puts("Hello, world!");

    return 0;
}
