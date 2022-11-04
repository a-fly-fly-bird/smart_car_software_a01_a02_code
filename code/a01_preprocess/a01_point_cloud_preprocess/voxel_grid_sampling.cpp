#include <iostream>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h> //随机采样
#include <pcl/filters/voxel_grid.h>    //体素滤波

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

// typedef pcl::PointXYZ PointT;
void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud);

int main(int argc, char *argv[])
{
    string cloud_path = "/home/ty/Desktop/resources/pcls/Horse.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(cloud_path.c_str(), *cloud);
    cout << "加载点云" << cloud->points.size() << "个" << endl;

    //创建滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    //设置输入点云
    vox.setInputCloud(cloud);
    //设置最小体素边长
    vox.setLeafSize(0.01f, 0.01f, 0.01f);
    //执行滤波采样
    vox.filter(*cloud_filtered);

    cout << "Cloud after fi: " << cloud_filtered->points.size() << endl;
    VisualizeCloud(cloud, cloud_filtered);

    return 0;
}

void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Voxel grid sampling"));

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
