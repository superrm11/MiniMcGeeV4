#pragma once
#include <atomic>
#include "nav_msgs/Odometry.h"

extern int fd;
extern std::atomic<int> x_mmps_setpt, theta_mmps_setpt;
extern nav_msgs::Odometry odom;