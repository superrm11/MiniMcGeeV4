#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_
#include "stdint.h"

typedef struct odometry_s
{ 
    int whealbase_mm;
    int wheel_diam_mm;
    int enc_cpr;
    int x_mm;
    int y_mm;
    int rot_deg;
    int64_t stored_l_enc;
    int64_t stored_r_enc;
} odometry_t;

void odometry_update(odometry_t* odom, int left_enc, int right_enc);

#endif