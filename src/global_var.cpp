#include "global_var.h"

int fd = -1;
std::atomic<int> x_mmps_setpt(0), theta_mmps_setpt(0);
nav_msgs::Odometry odom;
int line_sensor = 1000;

double deg2rad(double theta)
{
    return theta * PI / 180.0;
}

double rad2deg(double theta)
{
    return theta * 180.0 / PI;
}