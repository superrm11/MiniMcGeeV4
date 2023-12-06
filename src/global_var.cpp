#include "global_var.h"

int fd = -1;
std::atomic<int> x_mmps_setpt(0), theta_mmps_setpt(0);
nav_msgs::Odometry odom;