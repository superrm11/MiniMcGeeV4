#include "motor.h"
#include <stdlib.h>

/**
 * Initialize a Motor given pin 1 and 2 for a standard H-bridge setup
 * Works with L298 / L9110 / other H-bridge drivers
 * 
 * Initialize PWM on specified pins with a granularity of 1/127 (Forward pin)
 * and 1/127 (Reverse pin), and enable the signals. Default value is 0.
*/
motor_t *motor_init(uint pin1, uint pin2)
{
    // Initialize PWM for these GPIO
    gpio_set_function(pin1, GPIO_FUNC_PWM);
    gpio_set_function(pin2, GPIO_FUNC_PWM);

    uint pin_fwd_slice = pwm_gpio_to_slice_num(pin1);
    uint pin_rev_slice = pwm_gpio_to_slice_num(pin2);

    // Signed 8 bit max (-127 to 127)
    // Each pin gets 1/127 (or 1/127) of granularity
    pwm_set_wrap(pin_fwd_slice, 127);
    pwm_set_wrap(pin_rev_slice, 127);

    pwm_set_gpio_level(pin1, 0);
    pwm_set_gpio_level(pin2, 0);

    pwm_set_enabled(pin_fwd_slice, true);
    pwm_set_enabled(pin_rev_slice, true);

    // Initialize motor object defaulting to 0 speed
    motor_t *out = (motor_t *)malloc(sizeof(motor_t));
    out->pin_fwd = pin1;
    out->pin_rev = pin2;
    out->value = 0;

    return out;
}

/**
 * When finished, stop signals and free memory
*/
void motor_free(motor_t *motor)
{
    motor_set(motor, 0);
    free(motor);
}

/**
 * Set the motor either forward or reverse (-127 -> 127)
 * 0 is stopped
 * 
 * Follows generic H-bridge setup
*/
void motor_set(motor_t *motor, int8_t value)
{
    motor->value = value;
    
    // 0 speed: coast mode (no movement or resistance)
    if(value == 0)
    {
        pwm_set_gpio_level(motor->pin_fwd, 0);
        pwm_set_gpio_level(motor->pin_rev, 0);
    } else if (value > 0) // Forward
    {
        pwm_set_gpio_level(motor->pin_fwd, value);
        pwm_set_gpio_level(motor->pin_rev, 0);
    } else if(value < 0) // Reverse
    {
        pwm_set_gpio_level(motor->pin_fwd, 0);
        pwm_set_gpio_level(motor->pin_rev, -value);
    }
}