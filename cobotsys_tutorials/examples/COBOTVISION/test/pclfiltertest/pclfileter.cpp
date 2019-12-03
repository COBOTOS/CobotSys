//
// Created by cobot on 19-5-19.
//

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <logger/GlogConfig.h>
#include <logger/Logger.h>
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <VisionInputImageStruct.h>
#include <VisionFactoryInterface.h>
#include <VisionInterface.h>
#include <VisionGoodsInfoStruct.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/filters/passthrough.h>



#include<pcl/filters/passthrough.h>  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件


#include <ImageUtils.h>
#include <pcl/filters/project_inliers.h>


using namespace pcl;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_VISION_NS_API;
using namespace EAGLE_DATA_NS_COMMON;

template<typename PointT>
void showCloud( boost::shared_ptr< PointCloud<PointT> > cloud,std::string title) {
//    boost::thread([cloud,title]() {
        pcl::visualization::CloudViewer viewer(title);//直接创造一个显示窗口
        viewer.showCloud(cloud);//再这个窗口显示点云
        while (!viewer.wasStopped()) {
            sleep(2);
        }
//    }).detach();
}
/*
 * 直通滤波器：对于在空间分布有一定空间特征的点云数据，比如使用线结构光扫描的方式采集点云，
 * 沿z向分布较广，但x,y向的分布处于有限范围内。此时可使用直通滤波器，
 * 确定点云在x或y方向上的范围，可较快剪除离群点，达到第一步粗处理的目的。
 */
template<typename PointT>
boost::shared_ptr< PointCloud<PointT> > pclPassThrough(boost::shared_ptr< PointCloud<PointT> > cloud,float min,float max){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<PointT> passthrough;
    passthrough.setInputCloud(cloud);
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(min, max);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(false);//表示保留范围内， true表示保留范围外
    passthrough.filter(*cloud_after_PassThrough);
    return cloud_after_PassThrough;
}

/*
 * 体素滤波器：体素的概念类似于像素，使用AABB包围盒将点云数据体素化，一般体素越密集的地方信息越多，噪音点及离群点可通过体素网格去除。
 * 另一方面如果使用高分辨率相机等设备对点云进行采集，往往点云会较为密集。
 * 过多的点云数量会对后续分割工作带来困难。体素滤波器可以达到向下采样同时不破坏点云本身几何结构的功能。
 */
template<typename PointT>
boost::shared_ptr< PointCloud<PointT> > pclVoxelGrid(boost::shared_ptr< PointCloud<PointT> > cloud, float l, float w,
                                                     float h){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud(cloud);//输入点云数据
    voxelgrid.setLeafSize(l,w,h);//AABB长宽高
    voxelgrid.filter(*cloud_after_voxelgrid);
    return cloud_after_voxelgrid;
}

/*
 * 统计滤波器：考虑到离群点的特征，则可以定义某处点云小于某个密度，既点云无效。
 * 计算每个点到其最近的k个点平均距离。
 * 则点云中所有点的距离应构成高斯分布。给定均值与方差，可剔除3∑之外的点。
 */
template<typename PointT>
boost::shared_ptr< PointCloud<PointT> > pclStatisticalOutlierRemoval(boost::shared_ptr< PointCloud<PointT> > cloud, int MeanK,
        int thresh){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZ>);//

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;
    Statistical.setInputCloud(cloud);
    Statistical.setMeanK(MeanK);//取平均值的临近点数
    Statistical.setStddevMulThresh(thresh);//临近点数数目少于多少时会被舍弃
    Statistical.filter(*cloud_after_StatisticalRemoval);
    return cloud_after_StatisticalRemoval;
}

template<typename PointT>
boost::shared_ptr< PointCloud<PointT> > pclCondition(boost::shared_ptr< PointCloud<PointT> > cloud, int MeanK,
        int thresh){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Condition(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
                                                                                         pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));  //GT表示大于等于
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
                                                                                         pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));  //LT表示小于等于

    pcl::ConditionalRemoval<pcl::PointXYZ> condition;
    condition.setCondition(range_condition);
    condition.setInputCloud(cloud);                   //输入点云
    condition.setKeepOrganized(true);

    condition.filter(*cloud_after_Condition);
    return cloud_after_Condition;
}


void printPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl){
        for (size_t i = 0; i < pcl->points.size (); ++i)
        std::cerr << "    " << pcl->points[i].x << " "
                  << pcl->points[i].y << " "
                  << pcl->points[i].z << std::endl;
}


int main() {
    boost::shared_ptr<Camera3DFactoryInterface> camera3DFactoryInterface = Camera3DFactoryInterface::create();
    boost::shared_ptr<Camera3DInterface> camera = camera3DFactoryInterface->createCamera3D("comatrix");
    camera->open("CMOD0511026");
    Camera3DParam camera3DParam;
    camera3DParam.gain = 160;
    camera3DParam.r = 40;
    camera3DParam.g = 40;
    camera3DParam.b = 40;
    camera3DParam.hdrEnabled = true;
    camera->setCamera3DParam(camera3DParam);
    std::vector<boost::shared_ptr<VisionInputImage>> images;
    images = camera->captureSync();
    auto pcl = ImageUtils::cvMatToPcl(*images.at(1)->image.get());
    std::cout << "原始点云数据点数：" << pcl->points.size()<< std::endl;


    pcl = pclVoxelGrid(pcl,0.01f, 0.01f, 0.01f);
    std::cout << "pclVoxelGrid数据点数：" << pcl->points.size()<< std::endl;
    showCloud(pcl,"pclVoxelGrid");


    {
        /*
         * 填充ModelCoefficients的值，本例中我们使用一个ax+by+cz+d=0的平面模型，
         * 其中a=b=d=0，c=1，换句话说，也就是X-Y平面，用户可以任意定义PCL中支持的模型圆球、圆柱、锥形等进行投影滤波。
         */
        // Create a set of planar coefficients with X=Y=0,Z=1
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (pcl);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_projected);
        printPcl(cloud_projected);
        showCloud(cloud_projected,"cloud_projected");
    }

//    pcl = pclPassThrough(pcl,0.8f, 1.3f);
//    std::cout << "pclPassThrough数据点数：" << pcl->points.size()<< std::endl;
//    showCloud(pcl,"pclPassThrough");
//
//    pcl= pclStatisticalOutlierRemoval(pcl,20,10);
//    std::cout << "pclStatisticalOutlierRemoval点云数据点数：" << pcl->points.size()<< std::endl;
//    showCloud(pcl,"pclStatisticalOutlierRemoval");
//



    for(;;){
        sleep(5);
    }

}