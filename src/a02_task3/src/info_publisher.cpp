#include <iostream>

#include "ros/ros.h"
#include "a02_task3/car_basic_msg.h"

#include "ugv_sdk/scout/scout_base.hpp"
#include "ugv_sdk/scout/scout_types.hpp"

using namespace westonrobot;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "basic_info_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<a02_task3_demo::car_basic_msg>("/car_basic_msg", 1000);
    ros::Rate loop_rate(1);
    int count = 0;
    std::string device_name = "can0";
    int32_t baud_rate = 0;
    ScoutBase scout;
    scout.Connect(device_name, baud_rate);
    while (ros::ok())
    {
        auto state = scout.GetScoutState();
        std::cout << "-------------------------------" << std::endl;
        std::cout << "control mode: " << static_cast<int>(state.control_mode)
                  << " , base state: " << static_cast<int>(state.base_state)
                  << std::endl;
        a02_task3_demo::car_basic_msg msg;
        msg.header.seq = count;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "car_basic_msg";
        msg.front_light_state = state.front_light_state.mode;
        msg.battery_voltage = state.battery_voltage;
        msg.linear_velocity = state.linear_velocity;
        
        msg.angular_velocity = state.angular_velocity;
        msg.left_odometry = state.left_odometry;
        msg.right_odometry = state.right_odometry;
        // 注意，自定义消息rostopic echo 的时候需要 source devel/setup.bash
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
