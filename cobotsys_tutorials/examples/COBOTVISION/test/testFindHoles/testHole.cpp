#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <boost/shared_ptr.hpp>
#include <logger/GlogConfig.h>
#include <logger/Logger.h>
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <VisionInputImageStruct.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
//#include <EagleDataMacrosInterface.h>
#include <pcl/common/common.h>
#include <LSRStatic.h>
#include <boosttime/BoostTime.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "testHole.h"
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>


using namespace EAGLE_CAMERA3D_NS_API;
using namespace EAGLE_DATA_NS_API;
using namespace LION_LSR_NS_API;
using namespace std;
using namespace cv;
using namespace pcl;
//2d直径最小值像素值
double min_radius = 25;
//2d直径最大值像素值
double max_radius = 50;

cv::Rect ROI(600, 600, 1200, 1000);
//孔直径 mm
double Hole_Diameter_Min = 8;
double Hole_Diameter_Max = 10;
double Hole_Deep_Min = 14;
double Hole_Deep_Max = 16;

double Hole_Min_Points = 300;

//用过曝的方式获取孔的图片
boost::shared_ptr<VisionInputImage> captureHoleImage(boost::shared_ptr<Camera3DInterface> camera) {
    Camera2DParam camera2DParam;
    camera2DParam.gain = 220;
    camera2DParam.frameRate = 1;
    camera2DParam.exposeTime = 120000;
    camera->setCamera2DParam(camera2DParam);
    boost::shared_ptr<VisionInputImage> image = camera->capture2DSync();
    return image;
}


template<typename PointT>
void viewPose(boost::shared_ptr<PointCloud<PointT> > bigCloud,
              const PointT centralPoint, const pcl::Normal centralPointNormal, const std::string text) {

//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(text));
//    boost::shared_ptr<PointCloud<PointT> > cloud(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    cloud->push_back(centralPoint);
//    normals->push_back(centralPointNormal);
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 255, 0, 0);
//    //添加需要显示的点云数据
//    viewer->addPointCloud<PointT>(cloud, single_color, "sample cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> single_color3(normals, 0, 0, 255);
//    //添加需要显示的点云法向。cloud为原始点云模型，normals为法向信息，1表示需要显示法向的点云间隔，即每1个点显示一次法向，1５0表示法向长度。
//    viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud, normals, 1, 150, "normals");
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(bigCloud, 0, 255, 0);
//    //添加需要显示的点云数据
//    viewer->addPointCloud<PointT>(bigCloud, single_color2, "big cloud");
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce(10000);
//    }

}

void filterHole(cv::Mat &hole) {
//    hole = hole.clone();
    GaussianBlur(hole, hole, Size(19, 19), 2, 2);
    int channels = hole.channels();
    LOG_INFO << "channels " << channels;
//    for (auto row = 0; row < hole.rows; row++) {
//        for (auto column = 0; column < hole.cols; column++) {
////            hole.at<uchar>(row,column)=0;
//            int value = hole.at<uchar>(row, column);
//            if (value < 255) {
//                hole.at<uchar>(row, column) = 0;
//            }
//        }
//    }
//    threshold(hole, hole, 255, 255, THRESH_BINARY_INV);
    imshow("filterHole", hole);
    waitKey(0);
}

void contours(cv::Mat &hole) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
//    imshow("filterHole 1 ", hole);
    findContours(hole, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
    Mat resultImage = Mat::zeros(hole.size(), CV_8U);
//    drawContours(resultImage, contours, -1, Scalar(255, 0, 255));
    for (auto points:contours) {
        auto rrt = fitEllipse(points);//exception
//        LOG_INFO<<"fitEllipse center "<<rrt.center ;
        LOG_INFO << " boundingRect width " << rrt.boundingRect().width << "  " << rrt.boundingRect().height;
        if (rrt.boundingRect().width >= min_radius && rrt.boundingRect().width <= max_radius) {
            if (rrt.boundingRect().height >= min_radius && rrt.boundingRect().height <= max_radius) {
                LOG_INFO << "circle center is " << rrt.center << "   width:" << rrt.boundingRect().width << " height:"
                         << rrt.boundingRect().height;
                ellipse(resultImage, rrt, Scalar::all(255), 1, 8);
            }
        }
    }
//    int index =0;
//    for(auto vec :hierarchy){
//        LOG_INFO<<vec;
//        LOG_INFO<<contours.at(index++);
//    }
//    matchShapes
    imshow("ellipse", resultImage);
    waitKey(0);
}

double getDistance(Point pointO, Point pointA) {
    double distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}

std::vector<double> getRectBorderLength(Point2f points[]) {
    std::vector<double> values;
    Point p0 = points[0];
    for (int i = 1; i < 4; i++) {
        values.push_back(getDistance(p0, points[i]));
    }
    return values;
}

std::vector<boost::shared_ptr<Mat>> findHoles(Mat &holeMat) {
    std::vector<boost::shared_ptr<Mat>> masks;
    holeMat = holeMat(ROI);

    cv::imshow("after ROI ", holeMat);
    cv::waitKey(0);
    //把大于200直接替换为255，否则直接变成0. （把200以下灰色换成白色，否则直接变成纯黑色）
    threshold(holeMat, holeMat, 200, 255, THRESH_BINARY);
    cv::imshow("after threshold 1", holeMat);
    cv::waitKey(0);

    //模糊把纯黑色的点模糊开，以方便去噪点
    GaussianBlur(holeMat, holeMat, Size(5, 5), 3, 3);

    cv::imshow("GaussianBlur ", holeMat);
    cv::waitKey(0);

    //去掉模糊之后的噪点
    fastNlMeansDenoising(holeMat, holeMat, 17);
    cv::imshow("fastNlMeansDenoising", holeMat);
    cv::waitKey(0);
//    GaussianBlur(holeMat, holeMat, Size(19, 19), 2, 2);

//    threshold(holeMat, holeMat, 190, 255, THRESH_BINARY);
//
//    cv::imshow("after threshold THRESH_BINARY", holeMat);
//    cv::waitKey(0);

    //再次二值化 像素大于190保留，否则改成纯黑色
    threshold(holeMat, holeMat, 190, 255, THRESH_TOZERO);


    cv::imshow("after threshold 2", holeMat);
    cv::waitKey(0);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(holeMat, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

    Mat resultImage = Mat::zeros(holeMat.size(), CV_8U);
//    drawContours(resultImage, contours, -1, Scalar(255, 0, 255));


    for (auto points:contours) {
        if (points.size() >= 5) {
            auto rrt = fitEllipse(points);
            if (rrt.boundingRect().width >= min_radius && rrt.boundingRect().width <= max_radius &&
                rrt.boundingRect().height >= min_radius && rrt.boundingRect().height <= max_radius) {
                LOG_INFO << " boundingRect pass width " << rrt.boundingRect().width << "  "
                         << rrt.boundingRect().height;
                std::vector<std::vector<Point> > contours;
                contours.push_back(points);
                boost::shared_ptr<Mat> mask = boost::shared_ptr<Mat>(new Mat());
                *mask = Mat::zeros(holeMat.size(), CV_8U);
//            drawContours(*mask, contours, 0, Scalar(255));
                circle(*mask, rrt.center, rrt.boundingRect().width / 2.0, Scalar(255), -1);
                circle(resultImage, rrt.center, rrt.boundingRect().width / 2.0, Scalar(255), -1);
                masks.push_back(mask);
                cv::imshow("mask ", *mask);
                cv::waitKey(0);
            } else {
                LOG_INFO << " boundingRect fail width " << rrt.boundingRect().width << "  "
                         << rrt.boundingRect().height;
            }
        }
    }
    cv::imshow("resultImage ", resultImage);
    cv::waitKey(0);
    return masks;
}

template<typename PointT>
void showCloud(boost::shared_ptr<PointCloud<PointT> > cloud, std::string title) {
//    pcl::visualization::CloudViewer viewer(title);//直接创造一个显示窗口
//    viewer.showCloud(cloud);//再这个窗口显示点云
//    while (!viewer.wasStopped()) {
//        sleep(2);
//    }
}

template<typename PointT>
void matToPcl(Mat &cloud, boost::shared_ptr<PointCloud<PointT> > pointCloud) {
    for (int row = 0; row < cloud.rows; row++) {
        for (int col = 0; col < cloud.cols; col++) {
            PointXYZRC pointXYZ;
            pointXYZ.x = cloud.at<cv::Point3f>(row, col).x;
            pointXYZ.y = cloud.at<cv::Point3f>(row, col).y;
            pointXYZ.z = cloud.at<cv::Point3f>(row, col).z;
            pointXYZ.row = row;
            pointXYZ.col = col;
            if (pointXYZ.z <= 0)
                continue;
            pointCloud->points.push_back(pointXYZ);
        }
    }
}

int calculateRadius(boost::shared_ptr<PointCloud<PointXYZRC> > blockPC, PointXYZRC p_center, pcl::Normal normal) {
    pcl::PointCloud<PointXYZRC>::Ptr cloud_after_PassThrough(new pcl::PointCloud<PointXYZRC>);
    pcl::PassThrough<PointXYZRC> passthrough;
    passthrough.setInputCloud(blockPC);
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(p_center.z, p_center.z + 3000);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(false);//表示保留范围内， true表示保留范围外
    passthrough.filter(*cloud_after_PassThrough);

    viewPose(cloud_after_PassThrough, p_center, normal, "calculateRadius");

    PointXYZRC minPt, maxPt;
    pcl::getMinMax3D(*cloud_after_PassThrough, minPt, maxPt);
    int r = std::max<int>(maxPt.x - minPt.x, maxPt.y - minPt.y);
//    LOG_INFO << " radius is " << r << " row:" << minPt.row << ":" << minPt.col << "  col:" << maxPt.row << ":"
//             << maxPt.col;
    return r;
}

int calculateSurfaceZ(boost::shared_ptr<PointCloud<PointXYZRC> > blockPC, PointXYZRC &p_center, pcl::Normal normal) {
    pcl::PointCloud<PointXYZRC>::Ptr cloud_after_PassThrough(new pcl::PointCloud<PointXYZRC>);
    pcl::PassThrough<PointXYZRC> passthrough;
    passthrough.setInputCloud(blockPC);
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(p_center.z, p_center.z + 3000);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(true);//表示保留范围内， true表示保留范围外
    passthrough.filter(*cloud_after_PassThrough);
    viewPose(cloud_after_PassThrough, p_center, normal, "calculateSurfaceZ");

    Eigen::Vector4f center;
    pcl::compute3DCentroid(*cloud_after_PassThrough, center);
    return p_center.z;
}


double
calculateHoleDeep(boost::shared_ptr<PointCloud<PointXYZRC> > holeCloud, PointXYZRC p_center, pcl::Normal normal) {
    pcl::PointCloud<PointXYZRC>::Ptr cloud_after_PassThrough(new pcl::PointCloud<PointXYZRC>);
    pcl::RadiusOutlierRemoval<PointXYZRC> outrem;
    outrem.setInputCloud(holeCloud);
    outrem.setRadiusSearch(3);
    //半径范围内最少包含的点数
    outrem.setMinNeighborsInRadius(3);
    // apply filter
    outrem.filter(*cloud_after_PassThrough);

    PointXYZRC minPt, maxPt;
    pcl::getMinMax3D(*cloud_after_PassThrough, minPt, maxPt);
    return maxPt.z - minPt.z;
}


void processHoles(Mat &cloud, std::vector<boost::shared_ptr<Mat>> holes) {
    cloud = cloud(ROI);
    for (auto block:holes) {
        Mat blockCloud;
        cloud.copyTo(blockCloud, *block);
        boost::shared_ptr<PointCloud<PointXYZRC> > blockPC = boost::shared_ptr<PointCloud<PointXYZRC> >(
                new PointCloud<PointXYZRC>());
        matToPcl(blockCloud, blockPC);

        Eigen::Vector4f center;
        pcl::compute3DCentroid(*blockPC, center);

        PointXYZRC p_center;
        p_center.x = center(0);
        p_center.y = center(1);
        p_center.z = center(2);
        pcl::Normal normal;
        normal.normal[0] = 0;
        normal.normal[1] = 0;
        normal.normal[2] = 1;
        if (blockPC->points.size() > Hole_Min_Points) {
        double deep = calculateHoleDeep(blockPC, p_center, normal);
        viewPose(blockPC, p_center, normal, "blockPC");
//
            int z = calculateSurfaceZ(blockPC, p_center, normal);
            int row = calculateRadius(blockPC, p_center, normal);
            p_center.z = z;
            LOG_INFO <<"Hole position "<<p_center.x <<" "<< p_center.y<<" "<< p_center.z <<"  Radius is " << row << "    hole deep is " << deep;
        }
//        int z = calculateSurfaceZ(blockPC, p_center, normal);
//        LOG_INFO << "calculateSurfaceZ is " << z;
    }
}


int main() {
    boost::shared_ptr<Camera3DFactoryInterface> camera3DFactoryInterface = Camera3DFactoryInterface::create();
    boost::shared_ptr<Camera3DInterface> camera = camera3DFactoryInterface->createCamera3D("comatrix");
    camera->open("COMD0511019");
    //copy 防止内存被相机插件覆盖
    auto holeImage = captureHoleImage(camera)->image->clone();
//    cv::imshow("hole",*holeImage->image);

    auto holes = findHoles(holeImage);

    auto images = camera->captureSync();
    for (auto img:images) {
        if (img->type == ImageType::Cloud) {
            Mat cloud = *img->image;
            processHoles(cloud, holes);
        }
    }


}


