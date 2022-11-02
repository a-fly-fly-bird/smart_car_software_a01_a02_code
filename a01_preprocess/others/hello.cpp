#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"

ros::Publisher laser_pub;

void LaserCallBack(sensor_msgs::PointCloud2ConstPtr laser_cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_filterd(new pcl::PointCloud<pcl::PointXYZI>);
    // 将ros消息转换为pcl点云
    pcl::fromROSMsg(*laser_cloud_msg, *lasercloud_in);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(lasercloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 1000);
    pass.setNegative(false);
    pass.filter(*lasercloud_filterd);
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*lasercloud_filterd, cloud_filtered_msg);
    cloud_filtered_msg.header.stamp = ros::Time::now();
    cloud_filtered_msg.header.frame_id = "baselink";
    std::cout << "发布点云" << std::endl;
    laser_pub.publish(cloud_filtered_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "passthrough");
    ros::NodeHandle n;
    ros::Subscriber laser_sub = n.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, LaserCallBack);
    laser_pub = n.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 10);
    ros::spin();
    return 0;
}
