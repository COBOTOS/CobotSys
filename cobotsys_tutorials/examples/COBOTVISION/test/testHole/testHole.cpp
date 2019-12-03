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

double block_width = 290;
double block_height = 125;
//拍照参数 gain 200 grb 50 50 50  hdr off
cv::Rect ROI(600, 600, 1200, 1000);
//孔直径 mm
double Hole_Diameter_Min = 8;
double Hole_Diameter_Max = 12;
double Hole_Deep_Min = 14;
double Hole_Deep_Max = 16;

double Hole_Min_Points = 300;

//用过曝的方式获取孔的图片
boost::shared_ptr<VisionInputImage> captureHoleImage(boost::shared_ptr<Camera3DInterface> camera) {
    Camera2DParam camera2DParam;
    camera2DParam.gain = 220;
    camera2DParam.frameRate = 1;
//    camera2DParam.exposeTime = 90000;
//    camera2DParam.exposeTime = 120000;
    camera2DParam.exposeTime = 120000;
    camera->setCamera2DParam(camera2DParam);
    boost::shared_ptr<VisionInputImage> image = camera->capture2DSync();
    return image;
}


//用过曝的方式获取方块
boost::shared_ptr<VisionInputImage> captureBlockImage(boost::shared_ptr<Camera3DInterface> camera) {
    Camera2DParam camera2DParam;
    camera2DParam.gain = 200;
    camera2DParam.frameRate = 1;
    camera2DParam.exposeTime = 100000;
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


//void findBlock(Mat &block) {
//    imshow("block before GaussianBlur", block);
//    waitKey(0);
////    GaussianBlur(block, block, Size(19, 19), 12, 12);
////    imshow("block after GaussianBlur", block);
////    waitKey(0);
//
////    auto result = block.clone();
////    threshold(block, result, 30, 200.0, THRESH_BINARY_INV);
//    filterHole(block);
//    imshow("threshold ima ge", block);
//    waitKey(0);
//
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(block, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
////    cv::findContours(block, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
////    vector<vector<Point>> contours;
////    findContours(block, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
////    LOG_INFO<<" contours size "<<contours.size();
//    Mat resultImage1 = Mat::zeros(block.size(), CV_8U);
//    drawContours(resultImage1, contours, -1, Scalar(255, 0, 255));
//    imshow("resultImage1", resultImage1);
//    waitKey(0);
//    Mat resultImage = Mat::zeros(block.size(), CV_8U);
//    for (auto points : contours) {
//        cv::RotatedRect rotatedRect = cv::minAreaRect(points);
//
//        cv::Point2f rect_points[4];
//        rotatedRect.points(rect_points);
//        auto lengths = getRectBorderLength(rect_points);
//        std::sort(lengths.begin(), lengths.end());
//        double w = lengths.at(lengths.size() - 1);
//        double h = lengths.at(0);
//        LOG_INFO << "rotatedRect ::" << rotatedRect.angle << "   " << w << "    " << h;
//        if (w >= block_width * 0.9 && w <= block_width * 1.1) {
//            if (h >= block_height * 0.9 && h <= block_height * 1.1) {
//                float angle = rotatedRect.angle; // angle
//
//                // read center of rotated rect
//                cv::Point2f center = rotatedRect.center; // center
//
//                // draw rotated rect
//                for (unsigned int j = 0; j < 4; ++j)
//                    cv::line(resultImage, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 255));
//
//                // draw center and print text
//                std::stringstream ss;
//                ss << angle; // convert float to string
//                cv::circle(resultImage, center, 5, Scalar(255, 0, 255)); // draw center
//                cv::putText(resultImage, ss.str(), center + cv::Point2f(-25, 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
//                            Scalar(255, 0, 255)); // print angle
//            }
//        }
//    }
//    imshow("findBlock resultImage", resultImage);
//    waitKey(0);
//
//}

std::vector<boost::shared_ptr<Mat>> findHoles(Mat &holeMat) {
    std::vector<boost::shared_ptr<Mat>> masks;
    holeMat = holeMat(ROI);

    cv::imshow("after ROI ", holeMat);
    cv::waitKey(0);

    blur(holeMat, holeMat, Size(3, 3));
    imshow("blur ", holeMat);
    waitKey(0);

    int lowThreshold = 39, ratio = 3, kernel_size = 3;
    Canny(holeMat, holeMat, lowThreshold, lowThreshold * ratio, kernel_size);
    imshow("Canny ", holeMat);
    waitKey(0);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(holeMat, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

    Mat contourImage = Mat::zeros(holeMat.size(), CV_8U);
//    drawContours(contourImage, contours, -1, Scalar(255, 0, 255));
    LOG_INFO << "contours 1 size " << contours.size();
    for (auto points:contours) {
        if (points.size() >= 5) {
            auto rrt = fitEllipse(points);
            if (rrt.boundingRect().width >= min_radius && rrt.boundingRect().width <= max_radius &&
                rrt.boundingRect().height >= min_radius && rrt.boundingRect().height <= max_radius) {
//                LOG_INFO << " boundingRect pass width " << rrt.boundingRect().width << "  "
//                         << rrt.boundingRect().height;
                std::vector<std::vector<Point> > contours;
                contours.push_back(points);
//                boost::shared_ptr<Mat> mask = boost::shared_ptr<Mat>(new Mat());
//                *mask = Mat::zeros(holeMat.size(), CV_8U);
//            drawContours(*mask, contours, 0, Scalar(255));
//                circle(*mask, rrt.center, rrt.boundingRect().width / 2.0, Scalar(255), -1);
                circle(contourImage, rrt.center, rrt.boundingRect().width / 2.0, Scalar(255), -1);
//                masks.push_back(mask);
//                cv::imshow("mask ", *mask);
//                cv::waitKey(0);
            } else {
//                LOG_INFO << " boundingRect fail width " << rrt.boundingRect().width << "  "
//                         << rrt.boundingRect().height;
            }
        }
    }
    cv::imshow("contourImage1 ", contourImage);
    cv::waitKey(0);


    contours.clear();
    hierarchy.clear();
    findContours(contourImage, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
    contourImage = Mat::zeros(holeMat.size(), CV_8U);
    LOG_INFO << "contours 2 size " << contours.size();
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
                drawContours(*mask, contours, 0, Scalar(255));
                circle(*mask, rrt.center, rrt.boundingRect().width / 2.0, Scalar(255), -1);
                circle(contourImage, rrt.center, rrt.boundingRect().width / 2.0, Scalar(255), -1);
                masks.push_back(mask);
                cv::imshow("mask ", *mask);
                cv::waitKey(0);
            } else {
                LOG_INFO << " boundingRect fail width " << rrt.boundingRect().width << "  "
                         << rrt.boundingRect().height;
            }
        }
    }
    cv::imshow("contourImage2 ", contourImage);
    cv::waitKey(0);
    return masks;
}


std::vector<boost::shared_ptr<Mat>> findBlock(Mat &blockMat) {
    std::vector<boost::shared_ptr<Mat>> masks;
    blockMat = blockMat(ROI);
    GaussianBlur(blockMat, blockMat, Size(19, 19), 2, 2);
    threshold(blockMat, blockMat, 200, 255, THRESH_TOZERO);
    GaussianBlur(blockMat, blockMat, Size(19, 19), 2, 2);

    cv::imshow("after threshold ", blockMat);
    cv::waitKey(0);


    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(blockMat, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

    Mat resultImage1 = Mat::zeros(blockMat.size(), CV_8U);
    drawContours(resultImage1, contours, -1, Scalar(255, 0, 255));
    imshow("resultImage1", resultImage1);
    waitKey(0);

    for (auto points : contours) {
        cv::RotatedRect rotatedRect = cv::minAreaRect(points);

        cv::Point2f rect_points[4];
        rotatedRect.points(rect_points);
        auto lengths = getRectBorderLength(rect_points);
        std::sort(lengths.begin(), lengths.end());
        double w = lengths.at(lengths.size() - 1);
        double h = lengths.at(0);
        if (w >= block_width * 0.8 && w <= block_width * 1.2 && h >= block_height * 0.8 && h <= block_height * 1.2) {
            LOG_INFO << "rotatedRect pass   " << rotatedRect.angle << "   " << w << "    " << h;
            boost::shared_ptr<Mat> mask = boost::shared_ptr<Mat>(new Mat());
            *mask = Mat::zeros(blockMat.size(), CV_8U);

            float angle = rotatedRect.angle; // angle

            // read center of rotated rect
            cv::Point2f center = rotatedRect.center; // center

//            std::vector<std::vector<Point> > contours;
//            std::vector<Point> contour;
//            contour.push_back(rect_points[0]);
//            contour.push_back(rect_points[1]);
//            contour.push_back(rect_points[2]);
//            contour.push_back(rect_points[3]);
//            contours.push_back(contour);
//            drawContours(*mask, contours, 0, CV_RGB(255, 255, 255));
            Point PointArray[4];
            PointArray[0] = rect_points[0];
            PointArray[1] = rect_points[1];
            PointArray[2] = rect_points[2];
            PointArray[3] = rect_points[3];
            fillConvexPoly(*mask, PointArray, 4, Scalar(255));
            cv::imshow("mask ", *mask);
            cv::waitKey(0);
            masks.push_back(mask);
        } else {
            LOG_INFO << "rotatedRect fail   " << rotatedRect.angle << "   " << w << "    " << h;
        }
    }

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


void processBlock(Mat &cloud, std::vector<boost::shared_ptr<Mat>> blocks) {
    cloud = cloud(ROI);
    for (auto block:blocks) {
        Mat blockCloud;
        cloud.copyTo(blockCloud, *block);
        boost::shared_ptr<PointCloud<PointXYZRC> > pointCloud = boost::shared_ptr<PointCloud<PointXYZRC> >(
                new PointCloud<PointXYZRC>());
        matToPcl<PointXYZRC>(blockCloud, pointCloud);
        showCloud(pointCloud, "blockPC");
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
//    LOG_INFO<<"x value "<<maxPt.x - minPt.x<<"  y value "<<maxPt.y - minPt.y;
    int r = std::max<int>(maxPt.x - minPt.x, maxPt.y - minPt.y);
    return r;
}

int calculateSurfaceZ(boost::shared_ptr<PointCloud<PointXYZRC> > blockPC, PointXYZRC &p_center, pcl::Normal normal) {
    pcl::PointCloud<PointXYZRC>::Ptr cloud_after_PassThrough(new pcl::PointCloud<PointXYZRC>);
    pcl::PassThrough<PointXYZRC> passthrough;
    passthrough.setInputCloud(blockPC);
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(p_center.z, p_center.z + 300);//设置直通滤波器操作范围
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


void processHoles(Mat &cloud, std::vector<boost::shared_ptr<Mat>> holes, std::vector<PointXYZRC> &holePoses) {
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

            p_center.any1 = row;
            p_center.any2 = deep;

            double r = 0, c = 0;
            for (auto i = 0; i < blockPC->points.size(); i++) {
                r = r + blockPC->points.at(i).row;
                c = c + blockPC->points.at(i).col;
            }

            p_center.row = r / blockPC->points.size();
            p_center.col = c / blockPC->points.size();
            holePoses.push_back(p_center);

            LOG_INFO << "Hole position " << p_center.x << " " << p_center.y << " " << p_center.z << "  Radius is "
                     << row << "    hole deep is " << deep << " row at "<<p_center.row <<" col at "<<p_center.col;
        }
    }
}


int main() {
    boost::shared_ptr<Camera3DFactoryInterface> camera3DFactoryInterface = Camera3DFactoryInterface::create();
    boost::shared_ptr<Camera3DInterface> camera = camera3DFactoryInterface->createCamera3D("comatrix");
    camera->open("COMD0511019");
    //copy 防止内存被相机插件覆盖
//    auto holeImage = captureHoleImage(camera)->image->clone();
//    cv::imshow("hole",*holeImage->image);
//    auto blockImage = captureBlockImage(camera)->image->clone();
//    cv::imshow("block",*blockImage->image);

    std::vector<boost::shared_ptr<Mat>> holes;
//    auto blocks = findBlock(blockImage);

    auto images = camera->captureSync();
    Mat showImage;
    for (auto img:images) {
        if (img->type == ImageType::Mono) {
            Mat image = *img->image;
            showImage = image.clone();
            holes = findHoles(image);
        };
    }
    std::vector<PointXYZRC> holePoses;
    for (auto img:images) {
        if (img->type == ImageType::Cloud) {
            Mat cloud = *img->image;
//            processBlock(cloud, blocks);
            processHoles(cloud, holes, holePoses);
        }
    }

    std::vector<PointXYZRC> positivePoses;
    cvtColor(showImage, showImage, COLOR_BGR2RGBA);
    showImage = showImage(ROI);
    for (auto pose:holePoses) {

        if (pose.any1 >= Hole_Diameter_Min && pose.any1 <= Hole_Diameter_Max &&  pose.any2>=Hole_Deep_Min) {
            LOG_INFO << pose;
            positivePoses.push_back(pose);
            circle(showImage, Point(pose.col, pose.row), 3, Scalar(255, 0, 0), -1);
        }
    }


    imshow("showImage ", showImage);
    waitKey(0);





//    contours(hole);


//    cv::Mat block = blockImage(ROI);
//
//    findBlock(block);


}


