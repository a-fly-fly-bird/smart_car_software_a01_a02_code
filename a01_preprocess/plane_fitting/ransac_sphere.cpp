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
    float X_best = 0, Y_best = 0, Z_best = 0,R_best = 0;//最佳拟合球面面参数x,y,z,r
    vector<int> inliers_best;//最佳拟合点集
    boost::uniform_int<> real(0,RSample_pointsNum-1);//随机数范围
    while((iterNum<iter_maxNum) && inPlaneNum_max<=RSample_pointsNum){
        vector<int> inliers ={};//拟合域内点集
        int inPlaneNum_t =0;
        float x1=0,x2=0,x3=0,x4=0,y1=0,y2=0,y3=0,y4=0,z1=0,z2=0,z3=0,z4=0;
	float D=0;
	float a,b,c;
	float a1,b1,c1;
	float a2,b2,c2;
        while(1){//取四个随机点
            int rand_i_1 = real(gen);
            int rand_i_2 = real(gen);
            int rand_i_3 = real(gen);
            int rand_i_4 = real(gen);
             a = r_sample[rand_i_1].x - r_sample[rand_i_2].x; b = r_sample[rand_i_1].y - r_sample[rand_i_2].y; c = r_sample[rand_i_1].z - r_sample[rand_i_2].z;
             a1 = r_sample[rand_i_3].x - r_sample[rand_i_4].x; b1 = r_sample[rand_i_3].y - r_sample[rand_i_4].y; c1 = r_sample[rand_i_3].z - r_sample[rand_i_4].z;
             a2 = r_sample[rand_i_2].x - r_sample[rand_i_3].x; b2 = r_sample[rand_i_2].y - r_sample[rand_i_3].y; c2 = r_sample[rand_i_2].z - r_sample[rand_i_3].z;

            D=a * b1*c2 + a2 * b*c1 + c * a1*b2 - (a2*b1*c + a1 * b*c2 + a * b2*c1);//判断四个点是否在一个平面
            if(rand_i_1 != rand_i_2 != rand_i_3 !=rand_i_4 && D !=0){//3个点不是同一个点
                x1 = r_sample[rand_i_1].x;x2 = r_sample[rand_i_2].x;x3 = r_sample[rand_i_3].x;x4 = r_sample[rand_i_4].x;
                y1 = r_sample[rand_i_1].y;y2 = r_sample[rand_i_2].y;y3 = r_sample[rand_i_3].y;y4 = r_sample[rand_i_4].y;
                z1 = r_sample[rand_i_1].z;z2 = r_sample[rand_i_2].z;z3 = r_sample[rand_i_3].z;z4 = r_sample[rand_i_4].z;
                break;
            }
        }
        //求球心和半径
        float A = x1 * x1 - x2 * x2;
        float B = y1 * y1 - y2 * y2;
    	float C = z1 * z1 - z2 * z2;
	float A1 = x3 * x3 - x4 * x4;
	float B1 = y3 * y3 - y4 * y4;
	float C1 = z3 * z3 - z4 * z4;
	float A2 = x2 * x2 - x3 * x3;
	float B2 = y2 * y2 - y3 * y3;
	float C2 = z2 * z2 - z3 * z3;
	float P = (A + B + C) / 2;
	float Q = (A1 + B1 + C1) / 2;
	float R = (A2 + B2 + C2) / 2;
      
	float Dx = P * b1*c2 + b * c1*R + c * Q*b2 - (c*b1*R + P * c1*b2 + Q * b*c2);
        float Dy = a * Q*c2 + P * c1*a2 + c * a1*R - (c*Q*a2 + a * c1*R + c2 * P*a1);
        float Dz = a * b1*R + b * Q*a2 + P * a1*b2 - (a2*b1*P + a * Q*b2 + R * b*a1);
        //圆心坐标
	float center_x = Dx / D;
    	float center_y = Dy / D;
    	float center_z = Dz / D;
        //半径
	float radius = sqrt((x1 - center_x)*(x1 - center_x) +
        (y1 - center_y)*(y1 - center_y) +
        (z1 - center_z)*(z1 - center_z));




        /*float A_t = (y3 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float B_t = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
        float C_t = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
        float D_t = -(A_t * x1 + B_t * y1 + C_t * z1);
        float temp = sqrt(A_t*A_t+B_t*B_t+C_t*C_t);*/
        for(int i=0;i<RSample_pointsNum;i++){//遍历点云
            float temp_D = abs(sqrt((r_sample[i].x - center_x)*(r_sample[i].x - center_x) + (r_sample[i].y - center_y)*(r_sample[i].y - center_y) + (r_sample[i].z - center_z)*(r_sample[i].z - center_z))-radius);//点到圆心距离
            if(temp_D<maxD)//在域内
            {
                inPlaneNum_t++;//拟合域内点数+1
                inliers.push_back(i);//域内点进入inliers
            }
        }
        if(inPlaneNum_t>inPlaneNum_max)//如果域内点足够多
        {
            X_best = center_x;
            Y_best = center_y;
            Z_best = center_z;
            R_best = radius;
            inPlaneNum_max = inPlaneNum_t;
            inliers_best = inliers;
        }
        iterNum++;//一次迭代完成
    }
    return inliers_best;
}
int main(){
    string pcd_path = "/home/liujun/ransac/RANSAC_test_points/sphere.pcd";//pcd文件路径
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;//用于承载读出的点云
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
    vector<int> inliers = RANSAC(raw_cloud,max_nums,maxD);//拟合
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliter(new pcl::PointCloud<pcl::PointXYZ>);//用于承载拟合后的点云
    pcl::copyPointCloud(raw_cloud,inliers,*cloud_inliter);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//将raw_cloud转为指针raw_cloud_ptr，满足isualizercloud的使用。
    raw_cloud_ptr = raw_cloud.makeShared();
    VisualizeCloud(raw_cloud_ptr,cloud_inliter);
    return 0;
}

