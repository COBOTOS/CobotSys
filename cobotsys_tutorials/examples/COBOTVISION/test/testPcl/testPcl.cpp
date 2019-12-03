//
// Created by lijun on 19-3-27.
//
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
using namespace std;

void filter();


int main(int argc, char** argv)
{
#if 0
    int rows = 2048;
    int cols = 2448;
    std::chrono::high_resolution_clock::time_point tt0,tt1;
    tt0 = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    cloud->points.resize(rows * cols);
    tt1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> regTime=tt1-tt0;
    std::cout << "hello, world "<<regTime.count()<< std::endl;

    filter();
#endif
    Eigen::Vector3d planeNormal(2, 1, 3);
    Eigen::Vector3d nz(0, 0, 1);
    Eigen::Vector3d cNormal;
    cNormal = planeNormal.cross(nz);
    double dotResult = planeNormal.dot(nz);
    std::cout<<"cross result:"<<cNormal<<std::endl;
    std::cout<<"dot result:"<<dotResult<<std::endl;
    return 1;
}


void filter()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Z(new pcl::PointCloud <pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRemov(new pcl::PointCloud <pcl::PointXYZ>());
    ///---------------统计滤波
    pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_Z);
    sor.setMeanK(15);
    sor.setStddevMulThresh(0.1);//值越小过滤越明显
    sor.filter(*cloudRemov);
}