#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video_gray", 1);
        
        cv::namedWindow("input", 0);
        cv::namedWindow("output", 0);
    }

    ~ImageConverter()
    {
        cv::destroyWindow("input");
        cv::destroyWindow("output");
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat &input = cv_ptr->image;
        cv::Mat gray;

        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);

        cv::imshow("input", input);
        cv::imshow("output", gray);
        
        // Output modified video stream
        // 注意灰度图的编码格式是 sensor_msgs::image_encodings::MONO8， 不是原来图片的 BGR8 了！！！
        sensor_msgs::ImagePtr ros_gray = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, gray).toImageMsg();
        image_pub_.publish(ros_gray);

        cv::waitKey(40);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_gray_image_viewer");
    ImageConverter ic;
    ros::spin();
    return 0;
}