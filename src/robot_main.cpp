#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <atomic>
#include <wiringSerial.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "global_var.h"
#include "pidff.h"
#include "operations.h"
#include "moving_avg.h"

/**
 * Parse control data and send to pico
 * 
*/
void twist_callback(const geometry_msgs::Twist::ConstPtr& ros_data)
{
    if (enableTwistCallback)
    {
        x_mmps_setpt = ros_data->linear.x * 300;
        theta_mmps_setpt = ros_data->angular.z * 100;
    }
}

MovingAverage mav(20);

void image_callback(const sensor_msgs::ImageConstPtr& img_data)
{
    cv::Mat depth_img = cv_bridge::toCvShare(img_data)->image;
    printf("width: %d, height: %d\n", depth_img.cols, depth_img.rows);
    
    cv::Mat crop = depth_img(cv::Range(720-100, 720), cv::Range((1280/2)-50, (1280/2)+50));
    mav.update(cv::mean(crop).val[0]);
    printf("val: %f\n", mav.out);
    cam_val = mav.out;
    // cv::imshow("depth", crop * 16);
}

int main(int argc, char** argv)
{
    // ======== ROS initialization ========

    ros::init(argc, argv, "robot");

    ros::NodeHandle n("~");
    ros::Subscriber ctrl_sub = n.subscribe("/key_vel", 1000, twist_callback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);

    // cv::namedWindow("depth");
    // cv::startWindowThread();

    image_transport::ImageTransport it(n);
    image_transport::Subscriber it_sub = it.subscribe("/oak/stereo/image_raw", 1, image_callback);

    bool follow_line;
    bool retval = n.getParam("line", follow_line);

    if (!follow_line)
        printf("(_line:=false) Line Following Disabled!\n");
    else
        printf("(_line:=true) Line Following Enabled!\n");

    ros::Time last_time, cur_time;

    // ======== Serial initialization ========
    fd = serialOpen("/dev/ttyACM1", 115200);
    if(fd == -1)
    {
        perror("Unable to open Pico serial port!\n");
    }

    dist_fd = serialOpen("/dev/ttyACM0", 115200);
    if(dist_fd == -1)
    {
        perror("Unable to open Arduino serial port!\n");
    }

    // =============== Main Loop ===============
    ros::Rate r(10.0);
    while(n.ok() && ros::ok())
    {
        
        ros::spinOnce();

        #define BUFFER_LEN 80
        int i = 0;
        bool newData = false;
        // =============== Receive Data (From Arduino) ===============
        char buffer_dist[BUFFER_LEN];
        while(dist_fd != -1 && serialDataAvail(dist_fd) && i < BUFFER_LEN-1)
        {
            newData = true;
            buffer_dist[i++] = serialGetchar(dist_fd);
        }
        buffer_dist[i] = '\0';

        if(newData)
        {
            dist_sensor = atoi(buffer_dist);
            printf("Dist: %d\n", dist_sensor);
        }

        // =============== Receive Data (From Pico) ===============
        
        char buffer_pico[BUFFER_LEN];
        i = 0;
        newData = false;
        while(fd != -1 && serialDataAvail(fd) && i < BUFFER_LEN-1)
        {
            newData = true;
            buffer_pico[i++] = serialGetchar(fd);
        }
        buffer_pico[i] = '\0';


        if(newData)
        {
            #define TOKEN_LEN 20
            char* token[TOKEN_LEN];
            int i = 0;
            token[i] = strtok(buffer_pico, " ");
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

            printf("x:%d, y:%d, z:%d, dx:%d, dy:%d, dz:%d, l:%d, ",
                x, y, z, dx, dy, dz, l);

            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.orientation.z = z;
            odom.twist.twist.linear.x = dx;
            odom.twist.twist.linear.y = dy;
            odom.twist.twist.angular.z = dz;
            line_sensor = l;

            odom_pub.publish(odom);
        }
        // =============== Determine and Run Mode ===============
        if(follow_line)
            line_following();
        else
            mapping();


        // =============== Calculate Motor Velocities ===============
        // PID Units:
        //  kS: percent (Minimum to overcome friction)
        //  kV: percent per millimeter/sec (Linear)
        //  kP: percent per delta millimeter/sec measured
        //  kD: percent per delta millimeter measured
        //  kI: percent per delta millimeter/sec^2 measured

        static PIDFF x_pid, theta_pid;

        x_pid.target = x_mmps_setpt;

        x_pid.kS = 0;
        x_pid.kV = 0.18;  // measured 427 mmps at 80 speed
        x_pid.kP = 0.05; 
        x_pid.kI = 0; 
        x_pid.kD = 0;

        // Truncate to int, clamp 8 bit value
        x_pid.target = x_mmps_setpt;
        double angle_r = deg2rad(odom.pose.pose.orientation.z);
        double x = odom.twist.twist.linear.x;
        double y = odom.twist.twist.linear.y;
        double fwd_vel = (x * cos(angle_r)) + (y * sin(angle_r));
        printf("fwd_vel: %f, ", fwd_vel);
        int x_pid_out = (int)(x_pid.update(fwd_vel));

        theta_pid.kS = 0;
        theta_pid.kV = 0.5; // measured 200 deg/sec at 100 speed
        theta_pid.kP = 0;
        theta_pid.kI = 0;
        theta_pid.kD = 0;

        // Truncate to int, clamp by 100
        theta_pid.target = theta_mmps_setpt;
        int theta_pid_out = (int)(theta_pid.update(odom.twist.twist.angular.z));
        
        int left_motors = x_pid_out - theta_pid_out;
        int right_motors = x_pid_out + theta_pid_out;

        clamp(left_motors, -100, 100);
        clamp(right_motors, -100, 100);
        
        // =============== Send Data (To Pico) ===============

        printf("l: %d, r: %d\n", left_motors, right_motors);

        if(fd != -1)
        {
            serialPrintf(fd, "%d %d", left_motors, right_motors);
        }

        r.sleep();
    }
    
    // cv::destroyAllWindows();

    //TODO handle sigint / close serial & threads cleanly

}
