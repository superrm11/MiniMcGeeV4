#include "operations.h"
#include "global_var.h"
#include "pidff.h"

double smallest_angle(double start_deg, double end_deg)
{
  double retval;
  // get the difference between 0 and 360
  retval = fmod(end_deg - start_deg, 360.0);
  if(retval < 0)
    retval += 360.0;

  // Get the closest angle, now between -180 (turn left) and +180 (turn right)
  if(retval > 180)
    retval -= 360;
OdometryBase:
  return retval;
}

bool turn_deg(int deg, int speed_degps)
{
    static PIDFF pid;
    pid.kP = 0;
    pid.target = deg;

    static bool init = false;
    static int start_theta;

    int theta = odom.pose.pose.orientation.z;

    if(init == false)
    {
        start_theta = theta;
        init = true;
    }

    double angle_diff = smallest_angle(start_theta, theta);
    pid.update(angle_diff);
    theta_mmps_setpt = clamp(pid.out, -speed_degps, speed_degps);
    x_mmps_setpt = 0;

    if (fabs(angle_diff - deg) < 5)
    {
        theta_mmps_setpt = 0;
        x_mmps_setpt = 0;
        init = false;
        return true;
    }

    return false;
}

bool drive_mm(int x, int speed_mmps)
{
    static PIDFF pid;
    pid.kP = 0;
    pid.target = x;

    static bool init = false;
    static struct pose_s start_pose;

    int odom_x = odom.pose.pose.position.x;
    int odom_y = odom.pose.pose.position.y;

    if (init == false)
    {
        start_pose.x = odom_x;
        start_pose.y = odom_y;
        init = true;
    }

    struct pose_s displacement = {
        .x=(odom_x - start_pose.x), 
        .y=(odom_y - start_pose.y)
    };

    // Distance to the starting point
    double dist = sqrt((displacement.x * displacement.x) + (displacement.y * displacement.y));
    pid.update(dist);

    x_mmps_setpt = clamp(pid.out, -speed_mmps, speed_mmps);
    theta_mmps_setpt = 0;

    if(fabs(pid.error < 2))
    {
        x_mmps_setpt = 0;
        theta_mmps_setpt = 0;
        init = false;
        return true;
    }

    return false;
}

#define LINE_BASE_MMPS 50 //Tune

// Input: 0 -> 2000 (1000 = center)
// Ouptut: 0 -> max ()
PIDFF line_pid;

void line_following()
{
    int line_adjusted = line_sensor - 1000;

    line_pid.kS = 0;
    line_pid.kV = 0;
    line_pid.kP = 0;
    line_pid.kI = 0;
    line_pid.kD = 0;
    
    line_pid.target = 0;
    line_pid.update(line_adjusted);


}

void mapping()
{

}