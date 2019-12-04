//
// Created by Sail on 19-5-24.
// Copyright (c) 2018 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef SCREW_DETECTOR_H
#define SCREW_DETECTOR_H

#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/features/normal_3d_omp.h>
#include <omp.h>

#define DEBUG_SCREW

struct ObjectPose {
    pcl::PointXYZ center;//表面中心
    pcl::Normal normal; //从表面向外
};

struct detectParam {
    //roi set
    int roiX;
    int roiY;
    int roiW;
    int roiH;

    //点云过滤参数
    float ptMinZ;
    float ptMaxZ;
    float ptMinX;
    float ptMaxX;
    float ptMinY;
    float ptMaxY;

    //下采样
    float downSampeX;
    float downSampeY;
    float downSampeZ;

    //  SAC 分割
    int    SACmodel;
    double SACdistanceThreshold;
    int    SACmaxIterations;
    double SACnormalweight;

    //滤波statisticalFilter参数
    int meanK;
    float mulStd;

    //欧式距离分割，去除螺丝，保留背景，用于重建
    float segdistanceThreshold;
    float segBkgroundMinSize;
    float segBkgroundMaxSize;

    //滤波参数
    float filterRadius;
    float filterMinNum;

    //法向量计算参数
    double normalRadius;

    //后处理
    //计算螺丝平面与背景平面的夹角
    float maxAngleInDegreeWithBkground;

    //螺丝点云簇大小范围
    float segScrewdistanceThreshold;
    int segScrewMinSize;
    int segScrewMaxSize;

    //用于显示，与3D相机所匹配的2D彩色图
    cv::Mat Mat_2DCamera23DCamera;

    //分割参数
    //最小尺寸,允许的点云簇的最小尺寸
    int segMinSize;
    //最大尺寸,允许的点云簇的最大尺寸
    int segMaxSize;
    //半径阈值, 典型值0.5
    float segResidualThreshold;
    //曲率阈值, 典型值0.05　
    float segCurvatureThreshold;
    //角度阈值，以度为单位，典型值5度
    float segThetaThresholdInDegree;
    //邻域阈值，指定用包含多少个点的邻域
    int segNeighborNumber;
    //分割的距离阈值
    float segMaxDis;

    //平面拟合的距离阈值
    float distanceThreshold;
    //最大迭代次数
    int maxIterations;
    //内点比例阈值
    float planeInliersRatio;

    //后处理参数
    float maxAngleInDegreeWithZ;
    float widthThreshold;
    float heightThreshold;
};

class ScrewDetector {
public:
    ScrewDetector(std::string &paramFullPath);

    virtual ~ScrewDetector();

    bool processVisionImage(cv::Mat &incloudImg);

    std::vector<Eigen::Matrix4d> getScrewPose() const;

    Eigen::Vector4d getPlaneCoeff() const;

public:
    std::vector<double> bkgroundPlaneCoeff;//plane parameters
    std::vector<Eigen::Matrix4d> _screwPoses;// screw pose vector
    Eigen::Vector4d _planeCoeff;
    cv::Mat _maskImg;//螺丝在2D图像上的位置，从1开始，存值为螺丝编号
    cv::Mat _maskColorImg;//与上述maskImg对应的彩色图

    bool getImgwithMask(cv::Mat &inImg,cv::Mat &inmaskImg,cv::Mat &outImg);
    bool getScrewMaskNum(cv::Point &inpt2d,int &num);
    bool getColor2DImg(cv::Mat &inImg ,int outputHeight,int outputWidth ,cv::Mat &outImg);

    bool getptcloudformmat(cv::Mat &ptcloud2D , pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud3D);

private:
    bool init(std::string &paramFullPath);


    bool getPoseInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr &incloud ,cv::Mat &ptcloud2D ,std::vector<double> &planeParam,std::vector<ObjectPose> &screwParam);
    //配置参数
    bool setParam(const std::string &fileFullName);
    //初始数据下采样
    bool downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud);
    //半径滤波，去除噪点
    bool radiusfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud );

    //0.1 由相机原始数据转化成，便于通过2D图像查找的mat格式的点云
    void generatePointCloud(const std::string &path_xy, const std::string &path_pointcloud,cv::Mat &pointCloud);
    void matToPCD(const cv::Mat& pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool bSave=true);
    bool cropROI(const cv::Mat &imgOrg, const cv::Mat &pcOrg, cv::Mat &imgROI, cv::Mat &pcROI);

    //对输入点云进行裁剪，留下Xmin-Xmax/Ymin-Ymax/Zmin-Zmax范围内点云
    bool getRoi(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud);

    //seg分割
    pcl::PointIndices::Ptr getmodelindices(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::ModelCoefficients::Ptr &coefficients, int modeltype , double  distanceThred  );
    pcl::PointCloud<pcl::Normal>::Ptr getCloudNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr const &cloud );
    pcl::PointCloud<pcl::PointXYZ>::Ptr extracindices (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointIndices::Ptr inliers, bool negative);


    bool getPlaneInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ,pcl::PointIndices::Ptr &inliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &inlierscloud,float &inliersSizeRatios);

    //对点云分割，得到去除螺丝的背景点云
    bool getbkground(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &bkgroundcloud , pcl::PointCloud<pcl::PointXYZ>::Ptr &targetcloud);

    bool getScrewCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointIndices::Ptr &otherinliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFilter);
    //分割得到各个螺丝点云簇
    bool getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters);

    void searchPixelInCloudMat(const cv::Mat & Img,  const pcl::PointXYZ & inputPt, cv::Point &outputPt, cv::Point knownPt = cv::Point(0,0), int  adjacent = 0 );

    bool ptcloud2ptimg(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcloud ,cv::Mat &ptcloud2D,int maskNum);

    bool getScrewPose(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud , ObjectPose &objPoses);

    bool transform2DImg(cv::Mat &inImg ,cv::Mat &outImg, int outputHeight,int outputWidth ,cv::Mat &T,int T_Type = 0);

    void initParam(const detectParam &param);

    bool loadParamFile(const std::string &fileFullName, detectParam &param);

    detectParam _param;
};

#endif //SCREW_DETECTOR_H
