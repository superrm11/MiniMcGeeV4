#include <cstdio>
#include <thread>
#include <wiringPi.h>
#include <wiringSerial.h>

#include "ros/ros.h"
#include "minimcgeev4/MM4Ctrl.h"
#include "simple_watchdog.h"

// Pico Send Message Format:
// 8 bytes - Motor 1 Speed
// 8 bytes - Motor 2 Speed
// 8 bytes - Motor 3 Speed
// 8 bytes - Motor 4 Speed
// 1 bit - Reset Left Encoder
// 1 bit - Reset Right Encoder
// 6 bits - Unused
//
// Total: 5 bytes

#define PICO_SEND_MSG_SIZE 5

// Pico Receive Message Format:
// 4 bytes: Left Encoder Counter
// 4 bytes: Right Encoder Counter
//
// Total: 8 bytes

#define PICO_RECV_MSG_SIZE 8

int uartfd;


/**
 * Parse control data and send to pico
 * 
*/
void rosget_callback(const minimcgeev4::MM4CtrlConstPtr& ros_data)
{
    uint8_t send_data[PICO_SEND_MSG_SIZE];
    
    send_data[0] = ros_data->teleop_tank_L; // Front Left
    send_data[1] = ros_data->teleop_tank_L; // Back Left
    send_data[2] = ros_data->teleop_tank_R; // Front Right
    send_data[3] = ros_data->teleop_tank_R; // Back Right
    send_data[4] = 0b00000001 | 0b00000010; // Reset encoders: TODO
}

void uartget_callback()
{
    int data_index;
    uint8_t buffer[PICO_RECV_MSG_SIZE];
    while(true)
    {
        // TODO use ros realtime system? (spin())
        // 100hz constant INCLUDING processing

        data_index = 0;
        bool data_received = false;

        while(serialDataAvail(uartfd) > 0)
        {
            data_received = true;
            if(data_index >= PICO_RECV_MSG_SIZE)
            {
                //TODO send errors to ROS system
                perror("Incoming data is larger than expected message size! Flushing queue.");
                serialFlush(uartfd);
                break;
            }

            buffer[data_index++] = serialGetchar(uartfd);
        }

        if(data_received)
        {
            //Place into struct
        }

        //100 hz - 10ms
        usleep(10 * 1000);
    }
}

int main(int argc, char** argv)
{
    // ======== ROS initialization ========
    ros::init(argc, argv, "robot");
    ros::NodeHandle n;
    ros::Subscriber ctrl_sub = n.subscribe("control", 1000, rosget_callback);

    // ======== Serial initialization ========
    // Raspberry pi MUST have serial console disabled in raspi-config
    if (wiringPiSetup() < 0);
    {
        perror("Failed to set up WiringPi");
        return -1;
    }

    uartfd = serialOpen("/dev/serial0", 9600);
    std::thread uart_thread(uartget_callback);


    // Run the control listener
    ros::spin();

    //TODO handle sigint / close serial & threads cleanly

}
