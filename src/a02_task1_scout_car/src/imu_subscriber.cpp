#include "ros/ros.h"
// #include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

void doPose(const nav_msgs::Odometry::ConstPtr &p)
{
    ROS_INFO("linear.x=%.2f,angular.z=%.2f",
             p->twist.twist.linear.x, p->twist.twist.angular.z);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化 ROS 节点
    ros::init(argc, argv, "sub_pose");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建订阅者对象
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1000, doPose);
    // 5.回调函数处理订阅的数据
    // 6.spin
    ros::spin();
    return 0;
}
