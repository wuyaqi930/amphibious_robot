#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/common/transforms.h>
#include "pcl-1.7/pcl/point_cloud.h"
#include "pcl-1.7/pcl/PCLHeader.h"

#define PI 3.14159
#define HALF_PAI 1.5708

using namespace std;

//设置平移量为0
const double x = 0;
const double y = 0;
const double z = 0;

//对初始点云先绕z轴旋转90°
const double yaw0 = HALF_PAI;


//点云变换欧拉角
const double yaw = -HALF_PAI;
const double pitch = -HALF_PAI;
const double roll = -HALF_PAI;

//声明发布者
ros::Publisher pub;

//计算旋转平移矩阵
void compute_transform(Eigen::Matrix4f& m);


//将点云进行正确的坐标变换
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //输入pcl点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()); 

    //变换后的点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud_transformed;

    //ros消息类型转换为pcl数据类型
    pcl::fromROSMsg(*cloud_msg,*src_cloud);   

    //定义旋转平移矩阵并初始化
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

    //计算旋转平移矩阵
    compute_transform(transform_matrix);

    //进行点云变换
    pcl::transformPointCloud(*src_cloud, cloud_transformed, transform_matrix);

    sensor_msgs::PointCloud2 cloud_transformed_msg;
    pcl::toROSMsg(cloud_transformed, cloud_transformed_msg);
    pub.publish(cloud_transformed_msg);
}


int main (int argc, char** argv)
{

  ros::init (argc, argv, "pcl_transform");
  ros::NodeHandle n;

  //订阅gazebo中的点云信息，进行坐标变换并发布出去
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
 
  //发布坐标变换之后的点云信息
  pub = n.advertise<sensor_msgs::PointCloud2> ("point_transform", 1);
  ros::spin ();

}


//计算旋转平移矩阵
void compute_transform(Eigen::Matrix4f& m)
{
    //定义x,y,z分量旋转矩阵
    Eigen::Matrix3f rotation_z = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f rotation_y = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f rotation_x = Eigen::Matrix3f::Identity();

    //定义旋转矩阵
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    //对初始点云先绕z轴旋转90°
    Eigen::Matrix4f rotation_z_90 = Eigen::Matrix4f::Identity();
    rotation_z_90(0,0) = cos(yaw0);
    rotation_z_90(0,1) = -sin(yaw0);
    rotation_z_90(1,0) = sin(yaw0);
    rotation_z_90(1,1) = cos(yaw0);
    rotation_z_90(2,2) = 1;
    rotation_z_90(3,3) = 1;

   
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

    //计算最终旋转平移矩阵
    m = m * rotation_z_90;

    //打印最终旋转平移矩阵
    cout << "m = " << endl<< m << endl;
}