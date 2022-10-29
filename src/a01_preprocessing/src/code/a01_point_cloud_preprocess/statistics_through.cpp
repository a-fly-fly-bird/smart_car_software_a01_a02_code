#include <iostream>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

// typedef pcl::PointXYZ PointT;
void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud);

int main(int argc, char *argv[])
{
    string cloud_path = "/home/ty/Desktop/resources/pcls/Chair.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(cloud_path.c_str(), *cloud);
    cout << "加载点云" << cloud->points.size() << "个" << endl;

    // 声明创建统计滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_OutRemove;
    //输入点云
    sor_OutRemove.setInputCloud(cloud);
    //设置在进行统计时考虑查询点邻近点数
    sor_OutRemove.setMeanK(30);
    //设置判断是否为离群点的阈值
    sor_OutRemove.setStddevMulThresh(1.0);
    // 执行滤波返回滤波后的点云
    sor_OutRemove.filter(*cloud_filtered);

    cout << "Cloud after fi: " << cloud_filtered->points.size() << endl;
    VisualizeCloud(cloud, cloud_filtered);

    return 0;
}

void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Statistical filtering"));

    int v1(0), v2(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("point clouds", 10, 10, "v1_text", v1);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud", v1);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor_filtered(cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, fildColor_filtered, "cloud_filtered", v2);
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
