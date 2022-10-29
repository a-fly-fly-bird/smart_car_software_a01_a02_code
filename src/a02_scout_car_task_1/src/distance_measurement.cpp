#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

float total_distance = 0;
float previous_x = 0;
float previous_y = 0;
bool first_run = true;

void doMsg(const nav_msgs::Odometry::ConstPtr data)
{
    if (first_run)
    {
        previous_x = data->pose.pose.position.x;
        previous_y = data->pose.pose.position.y;
    }
    float x = data->pose.pose.position.x;
    float y = data->pose.pose.position.y;
    float d_increment = sqrt((x - previous_x) * (x - previous_x) + (y - previous_y) * (y - previous_y));
    total_distance = total_distance + d_increment;
    printf("Total distance traveled is %.2f m \r\n", total_distance);
    // pub.publish(data);
    previous_x = data->pose.pose.position.x;
    previous_y = data->pose.pose.position.y;
    first_run = false;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_sub");

    ros::NodeHandle nh_;

    ros::Subscriber sub_;

    sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, doMsg);

    ros::spin();

    return 0;
}
