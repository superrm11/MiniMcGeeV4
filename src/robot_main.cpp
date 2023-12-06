#include <cstdio>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <wiringSerial.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "global_var.h"
#include "pidff.h"

#define clamp(val, lower, upper) ((val < lower) ? lower : (val > upper) ? upper : val)

/**
 * Parse control data and send to pico
 * 
*/
void twist_callback(const geometry_msgs::Twist& ros_data)
{
    x_mmps_setpt = (int) ros_data.linear.x;
    theta_mmps_setpt = (int) ros_data.angular.z;
}

int main(int argc, char** argv)
{
    // ======== ROS initialization ========
    ros::init(argc, argv, "robot");

    ros::NodeHandle n;
    ros::Subscriber ctrl_sub = n.subscribe("cmd_vel", 1000, twist_callback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Time last_time, cur_time;

    // ======== Serial initialization ========
    fd = serialOpen("/dev/ttyACM0", 115200);
    if(fd == -1)
    {
        perror("Unable to open serial port!\n");
    }

    // =============== Main Loop ===============
    ros::Rate r(10.0);
    while(n.ok() && ros::ok())
    {
        
        ros::spinOnce();


        // =============== Receive Data (From Pico) ===============
        char x_buf[8];
        char y_buf[8];
        char rot_buf[8];
        char x_vel_buf[8];
        char y_vel_buf[8];
        char rot_vel_buf[8];
        char line_sense_buf[8];

        int i = 0, xi = 0, yi = 0, zi = 0, xvi = 0, yvi = 0, zvi = 0, lsi = 0;
        while(fd != -1 && serialDataAvail(fd))
        {
            char c = serialGetchar(fd);
            if (c == ' ')
                i++;
            else if (i == 1) // X
                x_buf[xi++] = c;
            else if (i == 3) // Y
                y_buf[yi++] = c;
            else if (i == 5) // Rot
                rot_buf[zi++] = c;
            else if (i == 7) // X mmps
                x_vel_buf[xvi++] = c;
            else if (i == 9) // Y mmps
                y_vel_buf[yvi++] = c;
            else if (i == 11) // Rot mmps
                rot_vel_buf[zvi++] = c;
            else if (i == 13) // Line Sensor
                line_sense_buf[lsi++] = c;
        }
        x_buf[xi] = '\0';
        y_buf[yi] = '\0';
        rot_buf[zi] = '\0';
        x_vel_buf[xvi] = '\0';
        y_vel_buf[yvi] = '\0';
        rot_vel_buf[zvi] = '\0';
        line_sense_buf[lsi] = '\0';

        int x = atoi(x_buf);
        int y = atoi(y_buf);
        int rot = atoi(rot_buf);
        int x_vel = atoi(x_vel_buf);
        int y_vel = atoi(y_vel_buf);
        int rot_vel = atoi(rot_vel_buf);
        int line_sense = atoi(line_sense_buf);

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation.z = rot;
        odom.twist.twist.linear.x = x_vel;
        odom.twist.twist.linear.y = y_vel;
        odom.twist.twist.angular.z = rot_vel;

        odom_pub.publish(odom);

        // =============== Calculate Motor Velocities ===============
        // PID Units:
        //  kS: 8-bit steps (Minimum to overcome friction)
        //  kV: 8-bit steps per millimeter/sec (Linear)
        //  kP: 8-bit steps per delta millimeter/sec measured
        //  kD: 8-bit steps per delta millimeter measured
        //  kI: 8-bit steps per delta millimeter/sec^2 measured

        static PIDFF x_pid, theta_pid;

        x_pid.target = x_mmps_setpt;

        x_pid.kS = 0; 
        x_pid.kV = 0; 
        x_pid.kP = 0; 
        x_pid.kI = 0; 
        x_pid.kD = 0;

        // Truncate to int, clamp 8 bit value
        int x_pid_out = (int)(x_pid.update(x_vel));

        theta_pid.kS = 0;
        theta_pid.kV = 0;
        theta_pid.kP = 0;
        theta_pid.kI = 0;
        theta_pid.kD = 0;

        // Truncate to int, clamp 8 bit value
        int theta_pid_out = (int)(theta_pid.update(rot_vel));

        int left_motors = x_pid_out - theta_pid_out;
        int right_motors = x_pid_out + theta_pid_out;

        uint8_t left_motors_u8 = clamp(left_motors, 0, 255);
        uint8_t right_motors_u8 = clamp(right_motors, 0, 255);
        
        // =============== Send Data (To Pico) ===============
        if(fd != -1)
        {
            char buffer[80];
            sprintf(buffer, "%u %u", left_motors_u8, right_motors_u8);
            serialPuts(fd, buffer);
        }

        r.sleep();
    }
    

    //TODO handle sigint / close serial & threads cleanly

}
