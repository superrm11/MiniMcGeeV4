#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "pico/stdlib.h"
#include "hardware/pwm.h"

typedef struct motor_s
{
    uint pin_fwd;
    uint pin_rev;
    int8_t value;
} motor_t;

/**
 * Initialize a Motor given pin 1 and 2 for a standard H-bridge setup
 * Works with L298 / L9110 / other H-bridge drivers
 * 
 * Initialize PWM on specified pins with a granularity of 1/100 (Forward pin)
 * and 1/100 (Reverse pin), and enable the signals. Default value is 0.
 * 
 * If motor is reversed, swap the pins' placement
 * 
 * @param pin1 "Forward" pin
 * @param pin2 "Reverse" pin
 * 
 * @return Pointer to motor data
*/
motor_t* motor_init(uint pin1, uint pin2);

/**
 * When finished, stop signals and free memory
 * 
 * @param motor Pointer to motor data
*/
void motor_free(motor_t *motor);

/**
 * Set the motor either forward or reverse (-100 -> 100)
 * 0 is stopped
 * 
 * Follows generic H-bridge setup
 * 
 * @param motor Pointer to motor data
 * @param value 8-bit speed (-100 to 100, 0 = stopped)
*/
void motor_set(motor_t *motor, int value);

#endif