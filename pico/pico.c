#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include "motor.h"
#include "as5600.h"

// MINIMCGEE PINOUT:
// Pin:     Function:
// 0        TX (Raspi4 RX)
// 1        RX (Raspi4 TX)
// 4        Left Rear PWM 1
// 5        Left Rear PWM 2
// 6        Right Rear PWM 1
// 7        Right Rear PWM 2
// 8        Left Front PWM 1
// 9        Left Front PWM 2
// 10       Right Front PWM 1
// 11       Right Front PWM 2
// 12       Right Encoder SDA
// 13       Right Encoder SCL
// 14       Left Encoder SDA
// 15       Left Encoder SCL

// Encoder Config 
#define L_ENC_I2C i2c1
#define L_ENC_SDA 14
#define L_ENC_SCL 15

#define R_ENC_I2C i2c0
#define R_ENC_SDA 12
#define R_ENC_SCL 13

// PWM Config
#define LF_PWM2 8 //LF backwards
#define LF_PWM1 9 //LF Forwards

#define RR_PWM1 4 //RR Forwards
#define RR_PWM2 5 //RR Backwards

#define RF_PWM2 10 //RF Backwards
#define RF_PWM1 11 //RF Forwards

#define LR_PWM2 6 //LR Backwards
#define LR_PWM1 7 //LR Forwards

motor_t *left_front, *left_rear, *right_front, *right_rear;


int main()
{
    stdio_init_all();
    puts("Hello, world!");

    left_front = motor_init(LF_PWM1, LF_PWM2);
    left_rear = motor_init(LR_PWM1, LR_PWM2);
    right_front = motor_init(RF_PWM1, RF_PWM2);
    right_rear = motor_init(RR_PWM1, RR_PWM2);

    bool is_left = true;
    int i = 0;

    while(true)
    {
        motor_set(left_front, 127);
        motor_set(left_rear, 127);
        motor_set(right_front, 127);
        motor_set(right_rear, 127);
        sleep_ms(1000);
        motor_set(left_front, 0);
        motor_set(left_rear, 0);
        motor_set(right_front, 0);
        motor_set(right_rear, 0);
        sleep_ms(1000);
        motor_set(left_front, -127);
        motor_set(left_rear, -127);
        motor_set(right_front, -127);
        motor_set(right_rear, -127);
        sleep_ms(1000);
        motor_set(left_front, 0);
        motor_set(left_rear, 0);
        motor_set(right_front, 0);
        motor_set(right_rear, 0);
        sleep_ms(1000);
    }

    return 0;
}
