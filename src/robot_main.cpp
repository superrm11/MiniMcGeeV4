#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <atomic>
#include <wiringSerial.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "global_var.h"
#include "pidff.h"

/**
 * Parse control data and send to pico
 * 
*/
void twist_callback(const geometry_msgs::Twist::ConstPtr& ros_data)
{
    x_mmps_setpt = ros_data->linear.x;
    theta_mmps_setpt = ros_data->angular.z;
}

int main(int argc, char** argv)
{
    // ======== ROS initialization ========
    ros::init(argc, argv, "robot");

    ros::NodeHandle n;
    ros::Subscriber ctrl_sub = n.subscribe("key_vel", 1000, twist_callback);
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
        #define BUFFER_LEN 80
        char buffer[BUFFER_LEN];
        int i = 0;
        bool newData = false;
        while(fd != -1 && serialDataAvail(fd) && i < BUFFER_LEN-1)
        {
            newData = true;
            buffer[i++] = serialGetchar(fd);
        }
        buffer[i] = '\0';


        if(newData)
        {
            #define TOKEN_LEN 20
            char* token[TOKEN_LEN];
            int i = 0;
            token[i] = strtok(buffer, " ");
            while(token[i++] != NULL && i < TOKEN_LEN)
            {
                token[i] = strtok(NULL, " ");
            }

            int x = atoi(token[0]);
            int y = atoi(token[1]);
            int z = atoi(token[2]);
            int dx = atoi(token[3]);
            int dy = atoi(token[4]);
            int dz = atoi(token[5]);
            int l = atoi(token[6]);

            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.orientation.z = z;
            odom.twist.twist.linear.x = dx;
            odom.twist.twist.linear.y = dy;
            odom.twist.twist.angular.z = dz;

            odom_pub.publish(odom);
        }

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
        // int x_pid_out = (int)(x_pid.update(x_vel));

        theta_pid.kS = 0;
        theta_pid.kV = 0;
        theta_pid.kP = 0;
        theta_pid.kI = 0;
        theta_pid.kD = 0;

        // Truncate to int, clamp 8 bit value
        // int theta_pid_out = (int)(theta_pid.update(rot_vel));
        
        // int left_motors = x_pid_out - theta_pid_out;
        // int right_motors = x_pid_out + theta_pid_out;

        // int8_t left_motors_i8 = clamp(left_motors, -127, 127);
        // int8_t right_motors_i8 = clamp(right_motors, -127, 127);
        
        // =============== Send Data (To Pico) ===============

        static int left = 0, right = 0;

        left = (x_mmps_setpt * 100) - (theta_mmps_setpt * 100);
        right = (x_mmps_setpt * 100) + (theta_mmps_setpt * 100);

        if(fd != -1)
        {
            serialPrintf(fd, "%d %d", left, right);
        }

        r.sleep();
    }
    

    //TODO handle sigint / close serial & threads cleanly

}
