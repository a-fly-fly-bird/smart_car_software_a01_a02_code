#include<iostream>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<boost/thread/thread.hpp>
#include<boost/random.hpp>
#include<string>
#include<pcl/visualization/pcl_visualizer.h>
using namespace std;
boost::random::mt19937 gen(time((time_t *)NULL));

//----------------------------点云可视化-----------------------------------------------------------------------
void VisualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
	//---------显示点云-----------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("point clouds", 10, 10, "v1_text", v1);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染,将z改为x或y即为按照x或y字段渲染
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud", v1);

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor_filtered(cloud, "z"); // 按照z字段进行渲染,将z改为x或y即为按照x或y字段渲染
	viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, fildColor_filtered,"cloud_filtered", v2);
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
//-----------------------拟合函数----------------------------------
vector<int> RANSAC (const pcl::PointCloud<pcl::PointXYZ>& r_sample,int iter_maxNum,float maxD){//点云------迭代次数------精度
    int RSample_pointsNum = r_sample.width,iterNum = 0;//点云大小，
    int inPlaneNum_max = 0;//拟合区域内点数
    float A_best = 0, B_best = 0, C_best = 0,D_best = 0;//最佳拟合平面参数
    vector<int> inliers_best;//最佳拟合点集
    boost::uniform_int<> real(0,RSample_pointsNum-1);//随机数范围
    while((iterNum<iter_maxNum) && inPlaneNum_max<=RSample_pointsNum){
        vector<int> inliers ={};//拟合域内点集
        int inPlaneNum_t =0;
        float x1=0,x2=0,x3=0,y1=0,y2=0,y3=0,z1=0,z2=0,z3=0;
        while(1){//取两个随机点,平面需要三个
            int rand_i_1 = real(gen);
            int rand_i_2 = real(gen);
            int rand_i_3 = real(gen);
            if(rand_i_1 != rand_i_2 != rand_i_3){//3个点不是同一个点
                x1 = r_sample[rand_i_1].x;x2 = r_sample[rand_i_2].x;x3 = r_sample[rand_i_3].x;
                y1 = r_sample[rand_i_1].y;y2 = r_sample[rand_i_2].y;y3 = r_sample[rand_i_3].y;
                z1 = r_sample[rand_i_1].z;z2 = r_sample[rand_i_2].z;z3 = r_sample[rand_i_3].z;
                break;
            }
        }
        float A_t = (y3 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float B_t = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
        float C_t = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
        float D_t = -(A_t * x1 + B_t * y1 + C_t * z1);
        float temp = sqrt(A_t*A_t+B_t*B_t+C_t*C_t);
        for(int i=0;i<RSample_pointsNum;i++){//遍历点云
            float temp_D = abs(A_t*r_sample[i].x + B_t*r_sample[i].y + C_t*r_sample[i].z + D_t)/temp;//点到平面距离
            if(temp_D<maxD)//在域内
            {
                inPlaneNum_t++;//拟合域内点数+1
                inliers.push_back(i);//域内点进入inliers
            }
        }
        if(inPlaneNum_t>inPlaneNum_max)//如果域内点足够多
        {
            A_best = A_t;
            B_best = B_t;
            C_best = C_t;
            D_best = D_t;
            inPlaneNum_max = inPlaneNum_t;
            inliers_best = inliers;
        }
        iterNum++;//一次迭代完成
    }
    return inliers_best;
}
int main(){
    string pcd_path = "/home/liujun/ransac/RANSAC_test_points/plane.pcd";
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::PCDReader reader;
    reader.read(pcd_path,raw_cloud);
    std::cout<<"--------raw_points----------"<<std::endl;
    std::cout<<raw_cloud.size()<<std::endl;
    float maxD = 0;
    int max_nums = 0;
    cout<<"input distance threshold:";
    cin>>maxD;
    cout<<"input the number of iterations:";
    cin>>max_nums;
    vector<int> inliers = RANSAC(raw_cloud,max_nums,maxD);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(raw_cloud,inliers,*cloud_inliter);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    raw_cloud_ptr = raw_cloud.makeShared();
    VisualizeCloud(raw_cloud_ptr,cloud_inliter);
    return 0;
}

