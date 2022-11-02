#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <algorithm_set/distance.h>

class ScoutDistanceMeasurement
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    float distance_;

public:
    ScoutDistanceMeasurement()
    {
        distance_ = 0.0;
        sub_ = nh_.subscribe("/odom", 1000, &ScoutDistanceMeasurement::doMsg, this);
    }

    void doMsg(const nav_msgs::Odometry::ConstPtr &msg)
    {
        float speed_x = msg->twist.twist.linear.x;
        ROS_INFO("X 方向的速度: %.4f (m)", speed_x);
        distance_ += algorithm_set::CalculatingDistanceByAverageSpeed(10, 0.04, speed_x);
        ROS_INFO("里程计的距离：%.4f (m)", distance_);
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "odom_sub");
    ScoutDistanceMeasurement scout_distance_measurement;
    ros::spin();
    return 0;
}