//#include <boost/shared_ptr.hpp>
//#include <logger/GlogConfig.h>
//#include <logger/Logger.h>
//#include <Camera3DFactoryInterface.h>
//#include <Camera3DInterface.h>
//#include <VisionInputImageStruct.h>
//#include <VisionFactoryInterface.h>
//#include <VisionInterface.h>
//#include <VisionGoodsInfoStruct.h>
//#include <opencv2/opencv.hpp>
//#include <boost/lexical_cast.hpp>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/common/transforms.h>
////#include "DetectorBaseSeg.h"
//
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/point_cloud_color_handlers.h>
//
////#include <EagleDataMacrosInterface.h>
//
//using namespace EAGLE_CAMERA3D_NS_API;
//using namespace EAGLE_DATA_NS_API;
//using namespace EAGLE_VISION_NS_API;
//
//typedef pcl::PointXYZ PointT;
//
//
//bool getMask(const cv::Mat &mask, std::vector<cv::Mat> &masks);
//bool computeSingleMask(const cv::Mat &mask, const int &label, cv::Mat &singleMask);
//void computeClusters(const std::vector<cv::Mat> &masks, const cv::Mat &pointCloud, std::vector<pcl::PointCloud<PointT>::Ptr> &clusters);
//bool computeSingleCluster(const cv::Mat &pointCloud, const cv::Mat &singleMask, pcl::PointCloud<PointT>::Ptr cluster);
//void showCloud(const pcl::PointCloud<PointT>::Ptr cloud, const std::string text);
//
//
//#define MM_2_M (1)
//
//int main(int argc, char **argv) {
//
//    cv::Mat img2d = cv::imread("/home/cobot/Desktop/bug/OK-for-collidebox-withvisionerror/Original2DImage.png");
//    cv::Mat pointCloud = cv::imread("/home/cobot/Desktop/bug/OK-for-collidebox-withvisionerror/ImageOfPointCloud.png");
//    cv::Mat mask = cv::imread("/home/cobot/Desktop/bug/OK-for-collidebox-withvisionerror/Image2DMask.png");
//
//    std::vector<cv::Mat> masks;
//    getMask(mask, masks);
//    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
//
//    computeClusters(masks, pointCloud, clusters);
//    for(int i = 0; i < masks.size(); i++)
//    {
//        std::string strName = "_" + std::to_string(i) + "xxxxxxxxxxxxxxxxxxx.png";
//        cv::imwrite(strName, masks[i]);
//    }
//
//    for(int i = 4; i < 5; i++)
////        for(int i = 0; i < clusters.size(); i++)
//
//        {
//        PointT minPt2, maxPt2;
//        float zSum = 0.0;
//        LOG_INFO << i<< "#__size="<< clusters[i]->points.size();
//
//        for(int j = 0; j < clusters[i]->points.size(); j++)
//        {
//            zSum = zSum+ clusters[i]->points[j].z;
//            LOG_INFO << clusters[i]->points[j].x << " "<< clusters[i]->points[j].y <<" " << clusters[i]->points[j].z;
//
//        }
//
//
//
//        float zMean = zSum /clusters[i]->points.size();
//        LOG_INFO << i<< "#__meanz="<< zMean;
////        pcl::getMinMax3D(*clusters[i], minPt2, maxPt2);
////        LOG_INFO << i<< "#__maxz="<< maxPt2.z << ";   minz=" << minPt2.z;
//        std::string title= "The " +std::to_string(i)  + "th cluster cloud";
//        showCloud(clusters[i],title);
//    }
//
//    return 1;
//}
//
//bool getMask(const cv::Mat &mask, std::vector<cv::Mat> &masks)
//{
//    LOG_INFO << "mask size: " << mask.rows << " ," << mask.cols;
//
//    if (mask.empty()) {
//        LOG_INFO << "ERROcdR:Det DetectorBaseSeg::getMask:gmask.empty()";
//        return false;
//    }
//
//    //将mask图像分解为每个图像的轮廓
//    double min, max;
//    cv::minMaxLoc(mask, &min, &max);
//    if (max < 1) {
//        LOG_INFO << "mask中无有效标记";
//        return false;
//    }
//    //生成各个物体的mask
//    for (int i = 1; i <= max; i++) {
//        cv::Mat singleMask;
//        bool res = computeSingleMask(mask, i, singleMask);
//        if (res) {
//            masks.push_back(singleMask);
//        }
//    }
//    LOG_INFO << "masks.size = " << masks.size();
//    LOG_INFO << "getMask get 2D masks: " << masks.size() << " of " << max << " raw ones";
//    return true;
//}
//
//bool computeSingleMask(const cv::Mat &mask, const int &label, cv::Mat &singleMask)
//{
//    cv::compare(mask, label, singleMask, cv::CMP_EQ);
//    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
//    cv::morphologyEx(singleMask, singleMask, cv::MORPH_OPEN, element);
//}
//
//void computeClusters(const std::vector<cv::Mat> &masks, const cv::Mat &pointCloud,
//                                      std::vector<pcl::PointCloud<PointT>::Ptr> &clusters)
//{
//    for (unsigned int i = 0; i < masks.size(); i++) {
//        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
//
//        LOG_INFO << "cluster ID:" << i;
//        bool res = computeSingleCluster(pointCloud, masks[i], cluster);
//        if (res) {
//            clusters.push_back(cluster);
//        }
//    }
//}
//
//bool computeSingleCluster(const cv::Mat &pointCloud, const cv::Mat &singleMask,
//                                           pcl::PointCloud<PointT>::Ptr cluster)
//{
//    cv::Mat maskObj;
//    LOG_INFO << "pointCloud.rows=" << pointCloud.rows  << ", " << "pointCloud.cols=" << pointCloud.cols ;
//    LOG_INFO << "singleMask.rows=" << singleMask.rows  << ", " << "singleMask.cols=" << singleMask.cols ;
//
//    pointCloud.copyTo(maskObj, singleMask);
//    LOG_INFO << "pointCloud copy OK";
//
//
//    //转化成点云格式
//    cluster->clear();
//    for (int i = 0; i < maskObj.rows; i++) {
//        for (int j = 0; j < maskObj.cols; j++) {
////                if (maskObj.at<cv::Vec3f>(i, j)[2] < _param.ptMinZ || maskObj.at<cv::Vec3f>(i, j)[2] > _param.ptMaxZ) {
//            if (maskObj.at<cv::Vec3f>(i, j)[2] < 600|| maskObj.at<cv::Vec3f>(i, j)[2] > 1500)
//            {
//                continue;
//            }
//            else
//            {
//                PointT point;
//                point.x = maskObj.at<cv::Vec3f>(i, j)[0];
//                point.y = maskObj.at<cv::Vec3f>(i, j)[1];
//                point.z = maskObj.at<cv::Vec3f>(i, j)[2];
//                cluster->points.push_back(point);
//            }
//        }
//    }
//
//    cluster->width = cluster->points.size();
//    cluster->height = 1;
//    cluster->is_dense = true;
//    return true;
//
//}
//
//void showCloud(const pcl::PointCloud<PointT>::Ptr cloud, const std::string text)
//{
//    typedef pcl::PointXYZ PointT;
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (text));
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 0, 255, 0);
//    //添加需要显示的点云数据
//    viewer->addPointCloud<PointT> (cloud, single_color, "Cloud");
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce(10000);
//    }
//}
