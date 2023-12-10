#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_
#include "stdint.h"

typedef struct odometry_s
{ 
    double whealbase_mm;
    double wheel_diam_mm;
    int enc_cpr;
    double x_mm;
    double y_mm;
    double rot_deg;
    double x_mmps;
    double y_mmps;
    double rot_degps;
    int64_t stored_l_enc;
    int64_t stored_r_enc;
} odometry_t;

void odometry_update(odometry_t* odom, int left_enc, int right_enc);

#endif