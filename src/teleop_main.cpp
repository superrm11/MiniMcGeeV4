#include "ros/ros.h"
#include "minimcgeev4/MM4Ctrl.h"

double clamp(double var)
{
    if (var > 1.0)
        return 1.0;
    else if (var < -1.0)
        return -1.0;
    
    return var;
}

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "teleop");
    ros::NodeHandle n;
    ros::Publisher ctrl_pub = n.advertise<minimcgeev4::MM4Ctrl>("control", 1000);
    ros::Rate rate(5);

    // Input initialization
    // gainput::InputManager imanager;
    
    // const gainput::DeviceId keyID = imanager.CreateDevice<gainput::InputDeviceKeyboard>();
    // const gainput::DeviceId conID = imanager.CreateDevice<gainput::InputDevicePad>();

    while(ros::ok())
    {
        minimcgeev4::MM4Ctrl msg;
        msg.auto_tank_L_mmps = 0;
        msg.auto_tank_R_mmps = 0;
        msg.teleop_tank_L = 0;
        msg.teleop_tank_R = 0;
        msg.en_teleop = false;

        // imanager.GetDevice(keyID)->Update(0);
        // ROS_INFO("Con Up? %d | Key Up? %d", imanager.GetDevice(conID)->IsAvailable(), imanager.GetDevice(keyID)->IsAvailable());
        // bool w = imanager.GetDevice(keyID)->GetBool(gainput::KeyW);
        // bool a = imanager.GetDevice(keyID)->GetBool(gainput::KeyA);
        // bool s = imanager.GetDevice(keyID)->GetBool(gainput::KeyS);
        // bool d = imanager.GetDevice(keyID)->GetBool(gainput::KeyD);

        double lstickY = 0, rstickX = 0;
        // if(imanager.GetDevice(conID)->IsAvailable())
        // {
        //     lstickY = imanager.GetDevice(conID)->GetFloat(gainput::PadButtonLeftStickY);
        //     rstickX = imanager.GetDevice(conID)->GetFloat(gainput::PadButtonRightStickX);
        // }

        // Arcade -> tank stick mixing
        // double con_tank_lside = clamp(-lstickY + rstickX);
        // double con_tank_rside = clamp(-lstickY - rstickX);

        // double key_tank_lside, key_tank_rside;
        // key_tank_lside = clamp(w - s + d - a);
        // key_tank_rside = clamp(w - s - d + a);

        // double lside_pct = clamp(key_tank_lside + con_tank_lside);
        // double rside_pct = clamp(key_tank_rside + con_tank_rside);

        // msg.teleop_tank_L = 127 * lside_pct;
        // msg.teleop_tank_R = 127 * rside_pct;

        // ROS_INFO("| CON | %f | %f | KEY | %f | %f | ", con_tank_lside, con_tank_rside, key_tank_lside, key_tank_rside);
        ctrl_pub.publish(msg);

        rate.sleep();
    }
}