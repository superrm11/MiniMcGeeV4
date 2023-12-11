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
    
  return retval;
}

bool turn_deg(int deg, int speed_degps)
{
    static PIDFF pid;
    // pid.kS = 15;
    pid.kP = 4;
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

bool turn_sec(double sec, int speed_degps)
{
    static bool init = false;
    static ros::Time start_time;
    if(!init)
    {
        start_time = ros::Time::now();
        init = true;
    }

    x_mmps_setpt = 0;
    theta_mmps_setpt = speed_degps;

    if((ros::Time::now() - start_time).toSec() > sec)
    {
        init = false;
        return true;
    }
    return false;
}

bool drive_sec(double sec, int speed_mmps)
{
    static bool init = false;
    static ros::Time start_time;
    if(!init)
    {
        start_time = ros::Time::now();
        init = true;
    }

    x_mmps_setpt = speed_mmps;
    theta_mmps_setpt = 0;

    if((ros::Time::now() - start_time).toSec() > sec)
    {
        init = false;
        return true;
    }
    return false;
}

bool drive_mm(int x, int speed_mmps)
{
    static PIDFF pid;
    // pid.kS = 15;
    pid.kP = 4;
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

void get_depth()
{
    
}

#define LINE_BASE_MMPS 110 //Tune

// Input: 0 -> 2000 (1000 = center)
// Line Sensor: 0 = Too Far Left! 2000 = Too Far Right!
PIDFF line_pid;

enum LineState 
{
    LINE, TURN1, DRIVE1, TURN2, DRIVE2, TURN3
};

LineState cur_state = LINE;

void line_following()
{
    enableTwistCallback = false;
    int line_adjusted = -(line_sensor - 1000);

    switch(cur_state)
    {
        case LINE:
        line_pid.kS = 0;
        line_pid.kV = 0;
        line_pid.kP = 0.02;
        line_pid.kI = 0;
        line_pid.kD = 0;
        
        line_pid.target = 0;
        line_pid.update(line_adjusted);

        static ros::Time tm = ros::Time::now();

        if(line_sensor == 0 || line_sensor == 2000)
        {
            if((ros::Time::now() - tm).toSec() > 0.5)
            {
                x_mmps_setpt = 0;
                theta_mmps_setpt = line_pid.out * 2.5;
            }
        }else
        {
            tm = ros::Time::now();
            x_mmps_setpt = LINE_BASE_MMPS - fabs(line_pid.error*.01);
            theta_mmps_setpt = line_pid.out;
        }

        // x_mmps_setpt = LINE_BASE_MMPS;
        // theta_mmps_setpt = 0;

        // if(dist_sensor < 120)
        //     cur_state = TURN1;

        break;
        case TURN1:
        if(turn_sec(0.15, 80))
            cur_state = DRIVE1;

        break;
        case DRIVE1:
        if(drive_sec(2, 120))
            cur_state = TURN2;

        break;
        case TURN2:
        if(turn_sec(0.5, -80))
            cur_state = DRIVE2;
        
        break;
        case DRIVE2:
        if(drive_sec(2, 120))
            cur_state = LINE;

        break;
        // case TURN3:
        // if(turn_sec(0.2, -80))
        //     cur_state = LINE;
        // break;
    }
    
    

}

void mapping()
{
    enableTwistCallback = true;

}