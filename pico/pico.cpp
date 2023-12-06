#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include "motor.h"
#include "as5600.h"
#include "odometry.h"
#include "QTRSensors.h"

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

#define QTR_EMIT 0
#define QTR_L 0
#define QTR_M 0
#define QTR_R 0

motor_t *left_front, *left_rear, *right_front, *right_rear;

odometry_t odom = {
    .whealbase_mm = 100,
    .wheel_diam_mm = 43,
    .enc_cpr = 4096,
    .x_mm = 0,
    .y_mm = 0,
    .rot_deg = 0,
    .x_mmps = 0,
    .y_mmps = 0,
    .rot_degps = 0
};

QTRSensors qtr;

int main()
{
    stdio_init_all();
    puts("Hello, world!");

    left_front = motor_init(LF_PWM1, LF_PWM2);
    left_rear = motor_init(LR_PWM1, LR_PWM2);
    right_front = motor_init(RF_PWM1, RF_PWM2);
    right_rear = motor_init(RR_PWM1, RR_PWM2);

    as5600_i2c_init(L_ENC_I2C, L_ENC_SDA, L_ENC_SCL);
    as5600_i2c_init(R_ENC_I2C, R_ENC_SDA, R_ENC_SCL);

    int64_t l_enc_ticks_abs = as5600_read_angle(L_ENC_I2C);
    int64_t r_enc_ticks_abs = as5600_read_angle(R_ENC_I2C);

    int l_enc_offset = l_enc_ticks_abs;
    int r_enc_offset = r_enc_ticks_abs;

    int64_t l_enc = 0;
    int64_t r_enc = 0;

    qtr.setTypeRC();
    qtr.setEmitterPin(QTR_EMIT);
    qtr.setSensorPins((const uint8_t[]){QTR_L, QTR_M, QTR_R}, 3);
    uint16_t sensor_vals[3];

    while(true)
    {
        as5600_get_continuous(L_ENC_I2C, &l_enc_ticks_abs);
        as5600_get_continuous(R_ENC_I2C, &r_enc_ticks_abs);

        l_enc = -l_enc_ticks_abs + l_enc_offset;
        r_enc = r_enc_ticks_abs - r_enc_offset;

        
        odometry_update(&odom, l_enc, r_enc);
        printf("%d %d %d %d %d %d %d\n", 
            odom.x_mm, odom.y_mm, odom.rot_deg,
            odom.x_mmps, odom.y_mmps, odom.rot_degps,
            qtr.readLineBlack(sensor_vals));


        // Read all characters available in the buffer
        char in = getchar_timeout_us(0);
        char buffer[80];
        int index = 0;
        while (in != PICO_ERROR_TIMEOUT)
        {
            // Avoid buffer overflow
            if (index >= 80)
                break;

            buffer[index++] = in;

            if(in == '\0')
                break;

            in = getchar_timeout_us(0);
        }

        int left_sp = 0;
        int right_sp = 0;

        char *token = strtok(buffer, " ");
        index = 0;
        while(token != NULL)
        {
            if (index == 0)
                left_sp = atoi(token);
            else if (index == 1)
                right_sp = atoi(token);
            
            token = strtok(NULL, " ");
            index++;
        }

        // uint16_t r_enc = as5600_read_angle(R_ENC_I2C);
        // uint16_t l_enc = as5600_read_angle(L_ENC_I2C);
        // printf("L Enc: %lld, R Enc: %lld\n ", l_enc, r_enc);
        // printf("left: %d, ri
        // char* tmp = 0;ght: %d\n", 0, r_enc);
        // motor_set(left_front, 127);
        // sleep_ms(1000);
        // motor_set(left_front, 0);
        sleep_ms(100);
    }

    return 0;
}