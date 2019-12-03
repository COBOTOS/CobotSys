/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-8-7           jianghao
============================================================== **/

#include <opencv2/opencv.hpp>
#include <systemproperty/SystemProperty.h>
#include <VisionInputImageStruct.h>
#include <EagleCamera3DApi.h>
#include <logger/Logger.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;

pcl::PointCloud<pcl::PointXYZ>::Ptr cvMatToPcl(cv::Mat &mat) {
    LOG_INFO << "finish reading file";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(
            new pcl::PointCloud<pcl::PointXYZ>);
    LOG_INFO << "begin parsing file";
    for (int ki = 0; ki < mat.rows; ki++) {
        for (int kj = 0; kj < mat.cols; kj++) {
            pcl::PointXYZ pointXYZ;

            pointXYZ.x = mat.at<cv::Point3f>(ki, kj).x;
            pointXYZ.y = mat.at<cv::Point3f>(ki, kj).y;
            pointXYZ.z = mat.at<cv::Point3f>(ki, kj).z;

            if(pointXYZ.z <= 0)
                continue;
            cloud->points.push_back(pointXYZ);
        }

    }
    return cloud;
}

void on_mouse(int EVENT, int x, int y, int flags, void* userdata) {
    cv::Mat hh;
    hh = *(cv::Mat *) userdata;
    cv::Point p(x, y);
    switch (EVENT) {
        case cv::EVENT_LBUTTONDOWN: {
            printf("b=%d\t", hh.at<cv::Vec3b>(p)[0]);
            printf("g=%d\t", hh.at<cv::Vec3b>(p)[1]);
            printf("r=%d\n", hh.at<cv::Vec3b>(p)[2]);
            cv::circle(hh, p, 2, cv::Scalar(255), 3);
            std::string ss;
            ss.append(std::to_string(x)).append(",").append(std::to_string(y));
            cv::putText(hh, ss, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 0), 2);
        }
            break;
    }
}

void show2D(const std::string& fileName){
    cv::namedWindow("display", cv::WINDOW_NORMAL);
    cv::Mat src;

    src = cv::imread(fileName);
    setMouseCallback("display", on_mouse, &src);
//以40ms刷新显示
    while (1)
    {
        cv::imshow("display", src);
        cv::waitKey(40);
    }
}

void show3D(const std::string& fileName){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>(fileName , *cloud);
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    LOG_INFO << "minX:" << minPt.x << " minY:" << minPt.y << " minZ:" << minPt.z;
    LOG_INFO << "maxX:" << maxPt.x << " maxY:" << maxPt.y << " maxZ:" << maxPt.z;
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);
    while(!viewer.wasStopped());
}

/* 参数格式
 *　②:显示2d图像 ./toolPegInHole 2D ../2d.png
 *  ③:显示3d点云 ./toolPegInHole 3D ../3d.ply
 *  ④:采集图片　 ./toolPegInHole comatrix COMD0511019 ../picPath/
 * */
int main(int argc, char** argv){
    if(argc == 3){
        std::string picType = argv[1];
        std::string fileName = argv[2];
        if(picType == "2D"){
            show2D(fileName);
        }else if(picType == "3D"){
            show3D(fileName);
        }
    }else if(argc > 3){
        std::string camType = argv[1];
        std::string camID = argv[2];
        std::string filePath = argv[3];
        boost::shared_ptr<Camera3DFactoryInterface> camera3DFactoryInterface = Camera3DFactoryInterface::create();
        boost::shared_ptr<Camera3DInterface> sptrCamera = camera3DFactoryInterface->createCamera3D(camType);
        sptrCamera->open(camID);
        std::vector<boost::shared_ptr<VisionInputImage>> images = sptrCamera->captureSync();
        for(auto item: images){
            if(item->type == ImageType::Color || item->type == ImageType::Mono){
                cv::imwrite(filePath + "2d.png", *item->image);
            }else if(item->type == ImageType::Cloud){
                auto cloud = cvMatToPcl(*item->image);
                pcl::io::savePLYFile(filePath + "3d.ply", *cloud);
            }
        }
    }
}