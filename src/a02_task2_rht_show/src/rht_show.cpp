#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try   // 如果转换失败，则提跳转到catch语句
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);   // 将图像转换openCV的格式，并输出到窗口
        cv::waitKey(1); // 一定要有wiatKey(),要不然是黑框或者无窗口     
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert for '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imageSub_node");   // 注册节点名 
    ros::NodeHandle nh; // 注册句柄
    image_transport::ImageTransport it(nh); // 注册句柄
    image_transport::Subscriber imageSub = it.subscribe("pro_image", 100, imageCallback);  // 订阅/cameraImage话题，并添加回调函数
    ros::spin();  // 循环等待回调函数触发
}
