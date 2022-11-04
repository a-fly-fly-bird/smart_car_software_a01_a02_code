#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/random.hpp>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

boost::random::mt19937 gen(time((time_t *)NULL)); // 随机数

void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Straight through filtering"));

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

vector<int> RANSAC(const pcl::PointCloud<pcl::PointXYZ> &r_sample, int iter_maxNum, float maxD)
{
    int RSample_pointsNum = r_sample.width, iterNum = 0;
    int inPlaneNum_max = 0;
    float A_best = 0, B_best = 0, C_best = 0, D_best = 0;
    vector<int> inliers_best;
    boost::uniform_int<> real(0, RSample_pointsNum - 1);
    while ((iterNum < iter_maxNum) && inPlaneNum_max <= RSample_pointsNum)
    {
        vector<int> inliers = {};
        int inPlaneNum_t = 0;
        float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        while (1)
        {
            int rand_i_1 = real(gen);
            int rand_i_2 = real(gen);
            if (rand_i_1 != rand_i_2)
            {
                x1 = r_sample[rand_i_1].x;
                x2 = r_sample[rand_i_2].x;
                y1 = r_sample[rand_i_1].y;
                y2 = r_sample[rand_i_2].y;
                break;
            }
        }
        float A_t = (y2 - y1);
        float B_t = (x1 - x2);
        float C_t = (x2 - x1) * y1 - x1 * (y2 - y1);
        float temp = sqrt(A_t * A_t + B_t * B_t);
        for (int i = 0; i < RSample_pointsNum; i++)
        {
            float temp_D = abs(A_t*r_sample[i].y + C_t) / temp;
            if(temp_D < maxD){
                inPlaneNum_t++;
                inliers.push_back(i);
            }
        }
        if(inPlaneNum_t >inPlaneNum_max){
            A_best = A_t;
            B_best = B_t;
            C_best = C_t;
            inPlaneNum_max = inPlaneNum_t;
            inliers_best=inliers;
        }
        iterNum++;
    }
    return inliers_best;
}

int main(){
    string pcd_path = "/home/ty/Desktop/resources/pcls/RANSAC/sphere.pcd";
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::PCDReader reader;
    reader.read(pcd_path, raw_cloud);
    cout << raw_cloud.size() << endl;
    float maxD = 0;
    int max_nums = 0;
    cout << "input distance threshold: ";
    cin >> maxD;
    cout << "input the number of iterations: ";
    cin >> max_nums;
    vector<int> inliters = RANSAC(raw_cloud, max_nums, maxD);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(raw_cloud, inliters, *cloud_inliter);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    raw_cloud_ptr = raw_cloud.makeShared();
    VisualizeCloud(raw_cloud_ptr, cloud_inliter);
    return 0;
}