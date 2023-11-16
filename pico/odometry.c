#include "odometry.h"
#include "stdint.h"
#include "math.h"

#define MEGAPI 3141593

void update(odometry_t* odom, int left_enc, int right_enc)
{
    // Math is adjusted to work well with integer math (avoid slow floating point)

    // Change in encoder value
    int l_enc_delta = left_enc - odom->stored_l_enc;
    int r_enc_delta = right_enc - odom->stored_r_enc;

    // No change, no need for calculations
    if(l_enc_delta == 0 && r_enc_delta == 0)
        return;

    odom->stored_l_enc = left_enc;
    odom->stored_r_enc = right_enc;

    // arclen = theta * r = (counts / cpr) * 2PI * (diam / 2)
    int64_t abs_delta_mm = (right_enc - left_enc) * odom->wheel_diam_mm * MEGAPI 
        / (2 * odom->enc_cpr * 1000000);

    // arclen = theta * r, theta = arclen / r
    // theta_rad = (delta) / (wheelbase / 2)
    int64_t theta_microrad = 1000000 * abs_delta_mm * odom->wheel_diam_mm / 2;
    uint16_t theta_deg = theta_microrad * 180 / MEGAPI;

    int rel_delta_x_mm = (l_enc_delta + r_enc_delta) * odom->wheel_diam_mm * MEGAPI
     / (2 * odom->enc_cpr * 1000000);
    // Forced to do floating point, blech
    int delta_x_mm = rel_delta_x_mm * cos(theta_microrad / 1000000.0);
    int delta_y_mm = rel_delta_x_mm * sin(theta_microrad / 1000000.0);

    odom->x_mm += delta_x_mm;
    odom->y_mm += delta_y_mm;
    odom->rot_deg = theta_deg;
}