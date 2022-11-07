#include<ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>     //直通滤波

using namespace std;

ros::Publisher cloud_pub;
ros::Publisher image_pub;
void projection(const sensor_msgs::PointCloud2::ConstPtr& input_cloud,const sensor_msgs::ImageConstPtr& input_image);
cv::Mat extrinsic_mat, camera_mat,dist_coeff; //外参矩阵，内参矩阵，畸变矩阵
cv::Mat rotate_mat,transform_vec; //旋转矩阵，平移向量
cv::FileStorage fs_read("/home/liujun/calibration/result.YM/result2", cv::FileStorage::READ); //打开标定结果文件
int main(int argc,char** argv){
    	    // 下列代码段从标定结果文件中读取外参矩阵、内参矩阵、畸变矩阵，
	    // 其中外参矩阵中：前三行前三列是旋转矩阵、第四列是平移向量
	   /* cv::Mat extrinsic_mat, camera_mat,dist_coeff; //外参矩阵，内参矩阵，畸变矩阵
        cv::Mat rotate_mat,transform_vec; //旋转矩阵，平移向量
	    cv::FileStorage fs_read("/home/liujun/calibration/result.YM/calibration_result", cv::FileStorage::READ); //打开标定结果文件*/
	    fs_read["CameraExtrinsicMat"] >> extrinsic_mat; //从文件里读取4x4外参矩阵
	    fs_read["CameraMat"] >> camera_mat; //从文件里读取3x3相机内参矩阵
	    fs_read["DistCoeff"] >> dist_coeff; //从文件里读取5x1畸变矩阵
	    fs_read.release(); //关闭文件

        	// 下列代码段从外参矩阵中读取旋转矩阵、平移向量
	rotate_mat=cv::Mat(3, 3, cv::DataType<double>::type); // 将旋转矩阵赋值成3x3矩阵
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            rotate_mat.at<double>(i,j)=extrinsic_mat.at<double>(j,i); // 取前三行前三列
        }
    }
    transform_vec=cv::Mat(3, 1, cv::DataType<double>::type); //将平移向量赋值成3x1矩阵
    transform_vec.at<double>(0)=extrinsic_mat.at<double>(1,3);
    transform_vec.at<double>(1)=extrinsic_mat.at<double>(2,3);
    transform_vec.at<double>(2)=-extrinsic_mat.at<double>(0,3);

    //node
    ros::init(argc,argv,"projection");
    ros::NodeHandle n;

    //multisubscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_cloud(n,"rslidar_points",10000,ros::TransportHints().tcpNoDelay());//订阅点云
    message_filters::Subscriber<sensor_msgs::Image> subscriber_image(n,"/camera/color/image_raw",10000,ros::TransportHints().tcpNoDelay());//订阅图像
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> syncPolicy;
	message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), subscriber_cloud, subscriber_image);  
	  	// 指定一个回调函数，就可以实现两个话题数据的同步获取
	sync.registerCallback(boost::bind(&projection, _1, _2));

    //publisher
    //image_transport::ImageTransport imageTransport(nh);
    //image_pub = imageTransport.advertise("pro_cloud", 1);
    


    image_pub = n.advertise<sensor_msgs::Image>("pro_image",10000);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("pro_cloud",10000);

    ros::spin();

    return 0;
}

// 该函数实现点云到图像的投影，将点云上的点投影到图像上并在图像上显示，并将能够投影到图像的点按图像对应像素的颜色对点进行染色
// 输入参数分别为点云和图像
void projection( const sensor_msgs::PointCloud2::ConstPtr& input_cloud,const sensor_msgs::ImageConstPtr& input_image)//回调函数
{
    /*pcl::PointCloud<pcl::PointXYZI>::Ptr ccloud(new pcl::PointCloud<pcl::PointXYZI>);//
    

    pcl::fromROSMsg (*input_cloud, *ccloud);//zhuanhuan
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input_image,  sensor_msgs::image_encodings::TYPE_8UC3);//zhuanhuan
   cv::Mat&img = cv_ptr->image;*/

   cv_bridge::CvImagePtr cv_ptr;//图像格式转换
            try {
                cv_ptr = cv_bridge::toCvCopy(input_image, "bgr8");
            }
            catch (cv_bridge::Exception &e) {
                return;
            }
            cv::Mat raw_img = cv_ptr->image;
	    cv::Mat img = raw_img.clone();

            pcl::PointCloud<pcl::PointXYZI>::Ptr ccloud(new pcl::PointCloud<pcl::PointXYZI>);//点云格式转换
	    pcl::PointCloud<pcl::PointXYZI>::Ptr xcloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*input_cloud, *xcloud);
	    pcl::PassThrough<pcl::PointXYZI> pt;	// 创建直通滤波器对象
	    pt.setInputCloud(xcloud);			//设置输入点云，这里设置的是一个指针
	    pt.setFilterFieldName("x");			//设置滤波所需字段x
	    pt.setFilterLimits(0.0, 10.0);		//设置x字段过滤范围
	    pt.setFilterLimitsNegative(false);	//默认false，保留范围内的点云；true，保存范围外的点云
	    //pt.setKeepOrganized(true);		//是否保持点云的组织结构（针对结构点云）
	    pt.filter(*ccloud);        //设置滤波输出的位置，这里设置的不是一个指针
            
            
          
    vector<cv::Point3f> points3d; //存储点云点的vcector，必须存储成cv::Point3f格式
    points3d.reserve(ccloud->size()+1); //先给points3d分配足够大的内存空间，避免push_back时频繁复制转移
//---------测试
    //std::cout<<"内存分配完毕"<<std::endl;
    cv::Point3f point;
    for(int i=0;i<ccloud->size();i++)
    {
        point.x=ccloud->points[i].x;
        point.y=ccloud->points[i].y;
        point.z=ccloud->points[i].z;
        points3d.push_back(point); //逐个插入
    }
//-----------测试
    //std::cout<<"插入完毕"<<std::endl;
    vector<cv::Point2f> projectedPoints; //该vector用来存储投影过后的二维点，三维点投影后变二维
    
    // 投影核心函数
    // 第一个参数为原始3d点
    // 第二个参数为旋转矩阵
    // 第三个参数为平移向量
    // 第四个参数为相机内参矩阵
    // 第五个参数为投影结果
    cv::projectPoints(points3d,rotate_mat,transform_vec,camera_mat,dist_coeff,projectedPoints);//投影
    //--------测试
    

    //pcl::PointXYZRGB point_rgb;//彩色像素点
	//遍历投影结果
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i<projectedPoints.size(); i++)
    {	
	pcl::PointXYZRGB point_rgb;//彩色像素点
	//std::cout<<"before p"<<std::endl;
        cv::Point2f p = projectedPoints[i];
	//std::cout<<"after p"<<std::endl;
	//------测试
	
	
        point_rgb.x=ccloud->points[i].x;
	//std::cout<<"x取值"<<std::endl;
        point_rgb.y=ccloud->points[i].y;
	//std::cout<<"y取值"<<std::endl;
        point_rgb.z=ccloud->points[i].z;
	//std::cout<<"z取值"<<std::endl;
        point_rgb.r=0;
        point_rgb.g=0;
        point_rgb.b=0;

	
		// 由于图像尺寸为640x480，所以投影后坐标不在图像范围内的点不进行染色（默认黑色）
       //改动swap
        if (p.y<480&&p.y>=0&&p.x<640&&p.x>=0) 
        {
	    
            point_rgb.r=int(img.at<cv::Vec3b>(p.y,p.x)[2]); //读取像素点的rgb值
	    
            point_rgb.g=int(img.at<cv::Vec3b>(p.y,p.x)[1]);
	    
            point_rgb.b=int(img.at<cv::Vec3b>(p.y,p.x)[0]);
	   
        }
	
        rgb_cloud->push_back(point_rgb); //对于投影后在图像中的点进行染色后加入点云rgb_cloud
    }
//-----------测试
   
    sensor_msgs::PointCloud2 ros_cloud; // 申明ros类型点云
    pcl::toROSMsg(*rgb_cloud,ros_cloud); // pcl点云转ros点云
    ros_cloud.header.frame_id="rslidar"; //
    cloud_pub.publish(ros_cloud); //cloud_pub是在函数外定义的一个ros点云发布器，将染色后的点云发布
   
    
    //再次遍历投影结果
    for (int i = 0; i<projectedPoints.size(); i++)
    {
        cv::Point2f p = projectedPoints[i];
        // 由于图像尺寸为640x480，所以投影后坐标处于图像范围内的点才在图像中进行标示
        if (p.y<480&&p.y>=0&&p.x<640&&p.x>=0)
        {
        	//标示的方式是在对应位置画圆圈
            cv::circle(img, p, 2, cv::Scalar(0, 0,255 ), 1, 8, 0);
        }
    }
//-----------测试
    //cv::imshow("view", img);
    //cv::waitKey(1);
    sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img).toImageMsg(); //利用cv_bridge将opencv图像转为ros图像
    image_pub.publish(msg); //image_pub是在函数外定义的一个ros图像发布器，将标示后的图像发布
    

}
