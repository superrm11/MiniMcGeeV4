#pragma once
#include <atomic>
#include "nav_msgs/Odometry.h"

#define PI 3.141592654
#define clamp(val, lower, upper) ((val < lower) ? lower : (val > upper) ? upper : val)

struct pose_s
{
    int x;
    int y;
    int r;
};

extern int fd, dist_fd;
extern std::atomic<double> x_mmps_setpt, theta_mmps_setpt;
extern nav_msgs::Odometry odom;
extern int line_sensor;
extern int dist_sensor;
extern bool enableTwistCallback;
extern double cam_val;

double deg2rad(double theta);
double rad2deg(double theta);