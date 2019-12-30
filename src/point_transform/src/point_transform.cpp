#include <iostream>
#include <cmath>
#include <ctime>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/common/transforms.h>
#include "pcl-1.7/pcl/point_cloud.h"
#include "pcl-1.7/pcl/PCLHeader.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#define PI 3.14159
#define HALF_PAI 1.5708

using namespace std;

//设置平移量为0
const double x = 0;
const double y = 0;
const double z = 0;

//对初始点云先绕z轴旋转90°
const double yaw0 = HALF_PAI;

//计算旋转平移矩阵
Eigen::Matrix4f compute_transform(double yaw,double pitch,double roll,double x,double y,double z);

//打印点云信息
void pcl_print( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//用类封装一个节点
class PointCloudTransform
{
    private:
    ros::NodeHandle n; 
    ros::Publisher pub;
    ros::Subscriber sub;   
    public:
    PointCloudTransform()
    {
        sub = n.subscribe("/camera/depth/points", 1000, &PointCloudTransform::cloud_Callback,this);
        pub = n.advertise<sensor_msgs::PointCloud2> ("/transformed_ros_cloud", 1000);
        
    }
        //将点云进行正确的坐标变换
    void cloud_Callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        clock_t starttime,endtime;
        starttime=clock();
        //输入pcl点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>()); 

        //降采样后的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());

        //变换后的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_base(new pcl::PointCloud<pcl::PointXYZ>());

        
        //ros消息类型转换为pcl数据类型
        pcl::fromROSMsg(*cloud_msg,*src_cloud);   

        
        //创建滤波对象
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(src_cloud);
        //设置栅格大小为 1x1x1cm
        filter.setLeafSize(0.01f, 0.01f, 0.1f);
        filter.filter(*filteredCloud);
        
        //定义旋转平移矩阵并初始化
        Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

        Eigen::Matrix4f transform_matrix_tobase = Eigen::Matrix4f::Identity();

        //计算旋转平移矩阵
        transform_matrix=compute_transform(-HALF_PAI,-HALF_PAI,-HALF_PAI,0,0,0)*compute_transform(HALF_PAI,0,0,0,0,0);

        //平移点云，参数要调
        transform_matrix_tobase=compute_transform(0,0.62832,0,-(0.250039208129731-0.012),-(0.0584396863666036-0.116),-(0.119264108531146+0.155));

        //在相机坐标系进行点云变换
        pcl::transformPointCloud(*filteredCloud, *cloud_transformed, transform_matrix);
        ROS_INFO("transformed height is [%d],width is[%d]",cloud_transformed->height,cloud_transformed->width);


        //打印点云在base_link下的坐标
        pcl::transformPointCloud(*cloud_transformed, *cloud_transformed_base, transform_matrix_tobase);
        pcl_print(cloud_transformed_base);

        sensor_msgs::PointCloud2 cloud_transformed_msg;
        pcl::toROSMsg(*cloud_transformed, cloud_transformed_msg);
        pub.publish(cloud_transformed_msg);
        endtime=clock();
        std::cout<<"time cost is "<<(double)(endtime-starttime)/CLOCKS_PER_SEC << " s" << std::endl;
    }
    
};


int main (int argc, char** argv)
{

  ros::init (argc, argv, "pcl_transform");

  //实例化对象
  PointCloudTransform PointProcesser;
 
  ros::spin ();

}


//计算旋转平移矩阵
Eigen::Matrix4f compute_transform(double yaw,double pitch,double roll,double x,double y,double z)
{

    Eigen::Matrix4f m= Eigen::Matrix4f::Identity();
    //定义x,y,z分量旋转矩阵
    Eigen::Matrix3f rotation_z = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f rotation_y = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f rotation_x = Eigen::Matrix3f::Identity();

    //定义旋转矩阵
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    rotation_x(0,0) = 1;
    rotation_x(1,1) = cos(roll);
    rotation_x(1,2) = -sin(roll);
    rotation_x(2,1) = sin(roll);
    rotation_x(2,2) = cos(roll);
 
    rotation_y(0,0) = cos(pitch);
    rotation_y(0,2) = sin(pitch);
    rotation_y(1,1) = 1;
    rotation_y(2,0) = -sin(pitch);
    rotation_y(2,2) = cos(pitch);

    rotation_z(0,0) = cos(yaw);
    rotation_z(0,1) = -sin(yaw);
    rotation_z(1,0) = sin(yaw);
    rotation_z(1,1) = cos(yaw);
    rotation_z(2,2) = 1;

    //计算旋转矩阵
    rotation = rotation_z * rotation_y * rotation_x;

    for (int i=0;i<3;i++)
    {
        for (int j=0;j<3;j++)
        {
            m(i,j) = rotation(i,j);
        }
    }

    //计算平移矩阵
    m(0,3) = -x;
    m(1,3) = -y;
    m(2,3) = -z;
    m(3,3) = 1;

    return m;
}
void pcl_print( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    
    std::cout<<"points.size:"<<cloud->points.size()<<std::endl;
    pcl::PointXYZ min,max;

    //计算三个方向坐标极值
    pcl::getMinMax3D(*cloud,min,max);
    //double x_min,y_min,z_min,x_max,y_max,z_max;
    /*
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        
        std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
        
    }
    */
     std::cerr<<"x_min:"<<min.x<<" "
     <<"x_max:"<<max.x<<endl;
     std::cerr<<"y_min:"<<min.y<<" "
     <<"y_max:"<<max.y<<endl;
     std::cerr<<"z_min:"<<min.z<<" "
     <<"z_max:"<<max.z<<endl;
    
}