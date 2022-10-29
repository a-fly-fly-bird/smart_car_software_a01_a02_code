// imu subscriiber.cpp

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <sensor_msgs/Imu.h>

void imu_callback(const sensor_msgs::Imu msg);

static double velocity_x_now = 0;

double calculate_line_velocity(double accel_x, double dt){
    return velocity_x_now + accel_x * dt;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_subscriber");

    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    ros::Subscriber imu_sub = n.subscribe("imu/data_raw", 100, imu_callback);

    ros::spinOnce();

    loop_rate.sleep();
}

void imu_callback(const sensor_msgs::Imu msg)
{
    printf("\033[2J\033[1;1H");

    printf("----------------------  开始输出  ----------------------\r\n");

    printf("朝向(orientation)数据是:            x: %8.3f    y: %8.3f   z: %8.3f  w: %8.3f\r\n", msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

    printf("角速度(angular_velocity)数据是:     x: %8.3f    y: %8.3f   z: %8.3f\r\n", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

    printf("线加速(linear_acceleration)数据是:   x: %8.3f    y: %8.3f  z: %8.3f\r\n", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

    velocity_x_now = calculate_line_velocity(msg.linear_acceleration.x, 0.01);

    printf("线速度是：  x: %8.3f ", velocity_x_now);
}
