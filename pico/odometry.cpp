#include "odometry.h"
#include "stdint.h"
#include "math.h"
#include "pico/time.h"
#include <cstdio>

#define MEGAPI 3141593
#define PI 3.141592654
#define TWOPI 6.283185308

absolute_time_t t_last;
int theta_last;

void odometry_update(odometry_t* odom, int left_enc, int right_enc)
{
    absolute_time_t t_cur = get_absolute_time();
    double time_delta_sec = us_to_ms(absolute_time_diff_us(t_last, t_cur)) / 1000.0;
    t_last = t_cur;
    
    // Change in encoder value
    double l_enc_delta = left_enc - odom->stored_l_enc;
    double r_enc_delta = right_enc - odom->stored_r_enc;

    // No change, no need for calculations
    if(l_enc_delta == 0 && r_enc_delta == 0)
        return;

    odom->stored_l_enc = left_enc;
    odom->stored_r_enc = right_enc;

    // Difference between left & right encoders ground travel in mm
    // wheel revs *  pi * wheel diameter
    double left_total_mm = left_enc * PI * odom->wheel_diam_mm / odom->enc_cpr;
    double right_total_mm = right_enc * PI * odom->wheel_diam_mm / odom->enc_cpr;
    double theta_arclen = (right_total_mm - left_total_mm ) / 2;

    // arclen = theta * r, theta = arclen / r
    double theta_rad = fmod(theta_arclen / (odom->whealbase_mm / 2), TWOPI);
    if(theta_rad < 0)
        theta_rad += TWOPI;

    double theta_deg = theta_rad * 180 / PI; 

    double avg_delta_mm = ((l_enc_delta + r_enc_delta) / (2 * odom->enc_cpr)) * PI * odom->wheel_diam_mm;

    double delta_x_mm = avg_delta_mm * cos(theta_rad);
    double delta_y_mm = avg_delta_mm * sin(theta_rad);

    odom->x_mm += delta_x_mm;
    odom->y_mm += delta_y_mm;
    odom->rot_deg = theta_deg;

    odom->x_mmps = delta_x_mm / time_delta_sec;
    odom->y_mmps = delta_y_mm / time_delta_sec;

    odom->rot_degps = (theta_deg - theta_last) / time_delta_sec;
    theta_last = theta_deg;

    // printf("x: %d, y: %d delta_t: %.2f\n", (int)odom->x_mmps, (int)odom->y_mmps, time_delta_sec);
    // printf("delta_x: %d, delta_y: %d\n", delta_x_mm, delta_y_mm);

}