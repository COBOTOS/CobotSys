//
// Created by Sail on 19-5-24.
// Copyright (c) 2018 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include "ScrewDetectorComatrix.h"

ScrewDetector::ScrewDetector(std::string &paramFullPath){

    bkgroundPlaneCoeff.resize(4);
    init(paramFullPath);
}

ScrewDetector::~ScrewDetector() {

}

bool ScrewDetector::processVisionImage(cv::Mat &ptcloud2D){
    std::cout<<"processVisionImage start"<<std::endl;
    std::vector<ObjectPose> screwsParam;
    std::vector<double> bkgroundPlaneParam;//plane parameters
    pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>);
    double time01 = cv::getTickCount();
    getptcloudformmat(ptcloud2D,incloud);
    double time02 = cv::getTickCount();

    bool result = getPoseInfo(incloud ,ptcloud2D,bkgroundPlaneParam,screwsParam);
    if(result){
        std::cout<<"screwsParam.size() = "<<screwsParam.size()<<std::endl;
        _planeCoeff<<bkgroundPlaneParam.at(0),bkgroundPlaneParam.at(1),bkgroundPlaneParam.at(2),bkgroundPlaneParam.at(3);
        std::cout<<"processVisionImage info"<<std::endl;
        std::cout<<"bkgroundPlaneParam info"<<std::endl;
        std::cout<<bkgroundPlaneParam.at(0)<<","<<bkgroundPlaneParam.at(1)<<","<<bkgroundPlaneParam.at(2)<<","<<bkgroundPlaneParam.at(3)<<std::endl;
        _screwPoses.clear();
        std::cout<<"screwsParam info"<<std::endl;
        for(auto poseV:screwsParam){
            std::cout<<"poseV info"<<std::endl;
            std::cout<<poseV.center.x<<","<<poseV.center.y<<","<<poseV.center.z<<std::endl;
            std::cout<<poseV.normal.normal_x<<","<<poseV.normal.normal_y<<","<<poseV.normal.normal_z<<std::endl;

            Eigen::Vector3d pos(poseV.center.x,poseV.center.y,poseV.center.z);
            Eigen::Vector3d normal(poseV.normal.normal_x,poseV.normal.normal_y,poseV.normal.normal_z);
            Eigen::Vector3d zAxis(poseV.normal.normal_x,poseV.normal.normal_y,poseV.normal.normal_z);
            Eigen::Vector3d xAxis,yAxis;
            xAxis<<1.0,0,0;
            yAxis=zAxis.cross(xAxis);
            xAxis=yAxis.cross(zAxis);
            Eigen::Matrix3d rotEigen;
            rotEigen<<xAxis,yAxis,zAxis;

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

            pose.block<3, 3>(0, 0) = rotEigen;
            pose.block<3, 1>(0, 3) = pos;
            _screwPoses.push_back(pose);
        }
    }
    double time03 = cv::getTickCount();

    std::cout<<"time01-02" << 1000*(time02-time01)/cv::getTickFrequency()<<std::endl;
    std::cout<<"time02-03" << 1000*(time03-time02)/cv::getTickFrequency()<<std::endl;

    return true;
}

std::vector<Eigen::Matrix4d> ScrewDetector::getScrewPose() const{
    return _screwPoses;
}

Eigen::Vector4d ScrewDetector::getPlaneCoeff() const{
    return _planeCoeff;
}

bool ScrewDetector::init(std::string &paramFullPath){
    bool res = setParam(paramFullPath);
    if (!res){
        std::cout<<"ScrewDetector::init() error"<<std::endl;
        return false;
    }
    return true;
}

bool ScrewDetector::ptcloud2ptimg(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcloud ,cv::Mat &ptcloud2D,int maskNum){
    std::vector<cv::Point> cluster2Dpoints;
    pcl::PointXYZ tmp_first;
    cv::Point pt2d_first;

    if (ptcloud->points.size() > 0){
        tmp_first = ptcloud->points[0];
        searchPixelInCloudMat(ptcloud2D,tmp_first,pt2d_first);
    }
    for (int i = 0; i < ptcloud->points.size(); ++i) {
        pcl::PointXYZ tmp = ptcloud->points[i];
        cv::Point pt2d;
        searchPixelInCloudMat(ptcloud2D,tmp,pt2d,pt2d_first,100);
        if (pt2d.y > 0 && pt2d.y < _maskImg.rows  && pt2d.x > 0 && pt2d.x < _maskImg.cols ){
            cluster2Dpoints.push_back(pt2d);
        }
    }
    std::vector<int> hull;
    cv::convexHull(cv::Mat(cluster2Dpoints),hull,false);//找离散点的边界
    cv::Point root_points[1][hull.size()];
    for (int i = 0; i < hull.size(); ++i) {
        root_points[0][i] = cluster2Dpoints[hull[i]];
    }
    const cv::Point* pt[1] = {root_points[0]};
    int npt[] = {(int)hull.size()};
    cv::fillPoly(_maskColorImg, pt, npt, 1, cv::Scalar(maskNum, maskNum, maskNum));
    return true;
}

bool ScrewDetector::getPoseInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr &incloud ,cv::Mat &incloudImg , std::vector<double> &planeParam,std::vector<ObjectPose> &screwParam){
    double time_getPoseInfo_000 = cv::getTickCount();

    getRoi(incloud);
    std::cout<<"Roi cloud size="<<incloud->points.size()<<std::endl;
    double time_getPoseInfo_001 = cv::getTickCount();
    std::cout<<"getPoseInfo getRoi cost time:"<<1000*(time_getPoseInfo_001 - time_getPoseInfo_000)/cv::getTickFrequency()<<std::endl;

    downsample(incloud);
    std::cout<<"downsample cloud size="<<incloud->points.size()<<std::endl;
    double time_getPoseInfo_002 = cv::getTickCount();
    std::cout<<"getPoseInfo downsample cost time:"<<1000*(time_getPoseInfo_002 - time_getPoseInfo_001)/cv::getTickFrequency()<<std::endl;


    radiusfilter(incloud);
    std::cout<<"radiusfilter cloud size="<<incloud->points.size()<<std::endl;
    double time_getPoseInfo_003 = cv::getTickCount();
    std::cout<<"getPoseInfo radiusfilter cost time:"<<1000*(time_getPoseInfo_003 - time_getPoseInfo_002)/cv::getTickFrequency()<<std::endl;

    //背景点云与其他（包括螺丝点云）
    pcl::PointCloud<pcl::PointXYZ>::Ptr bkgroundcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetcloud (new  pcl::PointCloud<pcl::PointXYZ>);

    //通过平面滤波获取背景点云//最大平面为墙面，其余中有螺丝
    pcl::PointIndices::Ptr backgroundIndices;
    pcl::ModelCoefficients::Ptr backgroundCoeff(new pcl::ModelCoefficients);

    std::cout<<"getmodelindices backgroundIndices"<<std::endl;
    backgroundIndices = getmodelindices(incloud,backgroundCoeff,0,3);//平面模型获取墙面内点及信息
    if (backgroundIndices->indices.size() < _param.segBkgroundMinSize){
        std::cout<<"墙面点云过少"<<std::endl;
        return false;
    }
    if(backgroundCoeff->values[2] > _param.ptMaxZ){
        std::cout<<"墙面距离过远"<<std::endl;
        return false;
    }
    //墙面平面参数 abcd
    if (backgroundCoeff->values[2] == 0){
        std::cout<<"背景平面参数计算错误"<<std::endl;
        return false;
    }
    if (backgroundCoeff->values[2] < 0){
        backgroundCoeff->values[0] = backgroundCoeff->values[0]*(-1);
        backgroundCoeff->values[1] = backgroundCoeff->values[1]*(-1);
        backgroundCoeff->values[2] = backgroundCoeff->values[2]*(-1);
        backgroundCoeff->values[3] = backgroundCoeff->values[3]*(-1);
    }
    bkgroundPlaneCoeff.push_back(backgroundCoeff->values[0]);
    bkgroundPlaneCoeff.push_back(backgroundCoeff->values[1]);
    bkgroundPlaneCoeff.push_back(backgroundCoeff->values[2]);
    bkgroundPlaneCoeff.push_back(backgroundCoeff->values[3]);

    std::cout<<"backgroundCoeff = "
               <<backgroundCoeff->values[0]<<","
               <<backgroundCoeff->values[1]<<","
               <<backgroundCoeff->values[2]<<","
               <<backgroundCoeff->values[3];

    planeParam.push_back(backgroundCoeff->values[0]);
    planeParam.push_back(backgroundCoeff->values[1]);
    planeParam.push_back(backgroundCoeff->values[2]);
    planeParam.push_back(backgroundCoeff->values[3]);

    double time_getPoseInfo_004 = cv::getTickCount();

    //分割出墙面与其他部分点云
    std::cout<<"backgroundIndices size="<<backgroundIndices->indices.size()<<std::endl;
    bkgroundcloud = extracindices(incloud,backgroundIndices,false);
    targetcloud = extracindices(incloud,backgroundIndices, true);
    std::cout<<"bkgroundcloud cloud size="<<bkgroundcloud->points.size()<<std::endl;
    std::cout<<"targetcloud cloud size="<<targetcloud->points.size()<<std::endl;
    std::cout<<"radiusfilter cloud size="<<incloud->points.size()<<std::endl;

    double time_getPoseInfo_005 = cv::getTickCount();

#ifdef DEBUG_SCREW
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_bkgroundcloud(new pcl::visualization::PCLVisualizer("bkgroundcloud"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_viewer_bkgroundcloud(bkgroundcloud, 255, 0, 0);
    viewer_bkgroundcloud->addPointCloud<pcl::PointXYZ> (bkgroundcloud, single_color_viewer_bkgroundcloud, "bkgroundcloud");
    while (!viewer_bkgroundcloud->wasStopped()){
        viewer_bkgroundcloud->spin();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_targetcloud(new pcl::visualization::PCLVisualizer("targetcloud"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_targetcloud(targetcloud, 255, 0, 0);
    viewer_targetcloud->addPointCloud<pcl::PointXYZ> (targetcloud, single_color_targetcloud, "targetcloud");
    while (!viewer_targetcloud->wasStopped()){
        viewer_targetcloud->spin();
    }
#endif

    //用欧式分割将每个cluster分割出来
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> screwclusters;
    getClusters(targetcloud,screwclusters);

    double time_getPoseInfo_006 = cv::getTickCount();

    //对每个cluster进行位姿估计，用平面模型估计
    std::vector<ObjectPose> clusterspose;
    ObjectPose clusterpose;
    _maskImg = cv::Mat::zeros(incloudImg.rows,incloudImg.cols,CV_8UC1);
    _maskColorImg = cv::Mat::zeros(incloudImg.rows,incloudImg.cols,CV_8UC3);

    int maskNum = 0;
    for (int i = 0; i < screwclusters.size(); ++i) {
        bool res = getScrewPose(screwclusters[i],clusterpose);
        if (res){
            maskNum ++ ;
            ptcloud2ptimg(screwclusters[i] , incloudImg, maskNum);//maskNum = 螺丝点云编号 + 1//后续便于将图像mask与点云位置对应
            clusterspose.push_back(clusterpose);
            screwParam.push_back(clusterpose);
        }
    }
    double time_getPoseInfo_007 = cv::getTickCount();


    std::cout<<"time_getPoseInfo_003-004:" << 1000*(time_getPoseInfo_004-time_getPoseInfo_003)/cv::getTickFrequency()<<std::endl;
    std::cout<<"time_getPoseInfo_004-005:" << 1000*(time_getPoseInfo_005-time_getPoseInfo_004)/cv::getTickFrequency()<<std::endl;
    std::cout<<"time_getPoseInfo_005-006:" << 1000*(time_getPoseInfo_006-time_getPoseInfo_005)/cv::getTickFrequency()<<std::endl;
    std::cout<<"time_getPoseInfo_006-007:" << 1000*(time_getPoseInfo_007-time_getPoseInfo_006)/cv::getTickFrequency()<<std::endl;

    cv::namedWindow("maskColorImg",cv::WINDOW_NORMAL);
    cv::imshow("maskColorImg",_maskColorImg);
    cv::waitKey(0);
    cv::cvtColor(_maskColorImg, _maskImg, cv::COLOR_RGB2BGR);
//    cv::imwrite("/home/cobot/demo/screw/0531/3D+2D/maskImg.bmp",_maskImg);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool ScrewDetector::getptcloudformmat(cv::Mat &ptcloud2D , pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud3D){
//#pragma omp parallel for
    for (int i = 0; i < ptcloud2D.rows; ++i) {
        for (int j = 0; j < ptcloud2D.cols; ++j) {
            pcl::PointXYZ tmp;
            tmp.x = ptcloud2D.at<cv::Vec3f>(i,j)[0];
            tmp.y = ptcloud2D.at<cv::Vec3f>(i,j)[1];
            tmp.z = ptcloud2D.at<cv::Vec3f>(i,j)[2];
            if (0 != tmp.z){
                ptcloud3D->points.push_back(tmp);
            }
        }
    }
    ptcloud3D->width = ptcloud3D->points.size();
    ptcloud3D->height = 1;
    ptcloud3D->is_dense = true;
    std::cout<<"ptcloud3D->points.size():"<<ptcloud3D->points.size()<<std::endl;
    return true;
}


bool passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<float> axisLimit,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudN) {

    pcl::PointIndicesPtr indices_z(new pcl::PointIndices), indices_zx(new pcl::PointIndices);
    pcl::PassThrough <pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(axisLimit[4], axisLimit[5]);
    pass.filter(indices_z->indices);

    pass.setIndices(indices_z);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(axisLimit[0],axisLimit[1]);
    pass.filter(indices_zx->indices);

    pass.setIndices(indices_zx);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(axisLimit[2],axisLimit[3]);
    pass.filter(*cloudN);
    return true;
}

bool ScrewDetector::getRoi(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud){
    std::vector<float> limit;
    limit.push_back(_param.ptMinX);
    limit.push_back(_param.ptMaxX);
    limit.push_back(_param.ptMinY);
    limit.push_back(_param.ptMaxY);
    limit.push_back(_param.ptMinZ);
    limit.push_back(_param.ptMaxZ);
    bool resFilter = passThroughFilter(oricloud, limit, oricloud);
    if(!resFilter){
        return false;
    }

#ifdef DEBUG_SCREW
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_roi(new pcl::visualization::PCLVisualizer("getRoi"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(oricloud, 255, 0, 0);
    viewer_roi->addPointCloud<pcl::PointXYZ> (oricloud, single_color0, "getRoi");
    while (!viewer_roi->wasStopped()){
        viewer_roi->spin();
    }
#endif
    return true;
}
bool downSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr orgCloud, double leafSizeX, double leafSizeY,
        double leafSizeZ, pcl::PointCloud<pcl::PointXYZ>::Ptr dCloud) {
    pcl::VoxelGrid <pcl::PointXYZ> filter;
    filter.setInputCloud(orgCloud);
    filter.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
    filter.filter(*dCloud);
    return true;
}

bool ScrewDetector::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud ){
    bool res = downSample(oricloud, _param.downSampeX,  _param.downSampeY,  _param.downSampeZ , oricloud);
    std::cout<<"toolKit downsample"<<oricloud->points.size()<<std::endl;

#ifdef DEBUG_SCREW
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_roi(new pcl::visualization::PCLVisualizer("downsample"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(oricloud, 255, 0, 0);
    viewer_roi->addPointCloud<pcl::PointXYZ> (oricloud, single_color0, "downsample");
    while (!viewer_roi->wasStopped()){
        viewer_roi->spin();
    }
#endif
}

bool radiusFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float FilterRadius, int MinNum,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(FilterRadius);
    //半径范围内最少包含的点数
    outrem.setMinNeighborsInRadius (MinNum);
    // apply filter
    outrem.filter (*cloud_filtered);
    return true;
}

bool ScrewDetector::radiusfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud ){

    bool res = radiusFilter(oricloud, _param.filterRadius, _param.filterMinNum, oricloud);
    std::cout<<"toolKit radiusfilter"<<oricloud->points.size()<<std::endl;

#ifdef DEBUG_SCREW
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_roi(new pcl::visualization::PCLVisualizer("downsample"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(oricloud, 255, 0, 0);
    viewer_roi->addPointCloud<pcl::PointXYZ> (oricloud, single_color0, "downsample");
    while (!viewer_roi->wasStopped()){
        viewer_roi->spin();
    }
#endif
}

bool ScrewDetector::getPlaneInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ,pcl::PointIndices::Ptr &inliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &inlierscloud,float &inliersSizeRatios){

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); //存储输出的模型的系数
    pcl::SACSegmentation<pcl::PointXYZ> seg; //可选设置
    seg.setOptimizeCoefficients (true); //必须设置
    seg.setModelType (pcl::SACMODEL_PLANE); //设置模型类型，检测平面
    seg.setMethodType (pcl::SAC_RANSAC); //设置方法【聚类或随机样本一致性】
    seg.setDistanceThreshold(4);
    seg.setInputCloud (cloud);
    seg.segment (*inliersIds, *coefficients); //分割操作

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < inliersIds->indices.size(); ++i){
        inlierscloud->points.push_back(cloud->points[inliersIds->indices[i]]);
    }
    inlierscloud->width = inlierscloud->points.size();
    inlierscloud->height = 1;
    inlierscloud->is_dense = true;


    inliersSizeRatios = inliersIds->indices.size()/cloud->points.size();

    return true;
}

void segmentEuclidean(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float MaxDis, int MinSize,
        int MaxSize, std::vector<pcl::PointIndices> &cluster_indices) {

    //分割，获取点索引
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(MaxDis);
    ec.setMinClusterSize(MinSize);
    ec.setMaxClusterSize(MaxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

bool statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int meanK, float mulStd, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFilter){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK); //K近邻搜索点个数
    sor.setStddevMulThresh(mulStd); //标准差倍数
    sor.setNegative(false); //保留未滤波点（内点）
    sor.filter(*cloudFilter);  //保存滤波结果到cloud_filter
}
bool ScrewDetector::getbkground(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &bkgroundcloud , pcl::PointCloud<pcl::PointXYZ>::Ptr &targetcloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(new pcl::PointCloud<pcl::PointXYZ>);
    bool res = statisticalFilter(cloud, _param.meanK, _param.mulStd, cloudFilter);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer01(new pcl::visualization::PCLVisualizer("statisticalFilter"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color01(cloudFilter, 255,255,255);
    viewer01->addPointCloud<pcl::PointXYZ> (cloudFilter, single_color01, "statisticalFilter");

    if(!res)
        return false;
    //利用欧式聚类分割出各个点云簇，去除螺丝，保留其他
    std::vector<pcl::PointIndices> clusterIndices;
    segmentEuclidean(cloudFilter, _param.segdistanceThreshold ,_param.segBkgroundMinSize,_param.segBkgroundMaxSize,clusterIndices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter2(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0(new pcl::visualization::PCLVisualizer("newCluster_inliers"));

    for (int i = 0; i < clusterIndices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int jj = 0; jj < clusterIndices[i].indices.size(); jj++) {
            newCluster->points.push_back(cloudFilter->points[clusterIndices[i].indices[jj]]);
        }
        newCluster->width = newCluster->points.size();
        newCluster->height = 1;
        newCluster->is_dense = true;

        *cloudFilter2 = *cloudFilter2 + *newCluster;

        pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster_inliers(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster_outliers(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndices::Ptr inliersIds (new pcl::PointIndices);
        float ratio = 0;
        getPlaneInliers(newCluster,inliersIds,newCluster_inliers,ratio);

        int r = rand()*255;
        int g = rand()*255;
        int b = rand()*255;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorin(newCluster_inliers, r, g, b);
        std::string str;
        str = "cloud" + std::to_string(r+g+b);
        viewer0->addPointCloud<pcl::PointXYZ> (newCluster_inliers, single_colorin, str);

        *bkgroundcloud = *bkgroundcloud + *newCluster_inliers;

        getScrewCloud(newCluster,inliersIds,newCluster_outliers);
        *targetcloud = *targetcloud + *newCluster_outliers;
    }
    cloud = cloudFilter;

#ifdef DEBUG_SCREW
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("getbkground clustercloud"));
    for (int i = 0; i < clusterIndices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int jj = 0; jj < clusterIndices[i].indices.size(); jj++) {
            newCluster->points.push_back(cloudFilter->points[clusterIndices[i].indices[jj]]);
        }
        newCluster->width = newCluster->points.size();
        newCluster->height = 1;
        newCluster->is_dense = true;
        std::ostringstream ost1;
        ost1 << "cluster_preprocessed_" << i << ".ply";

        int r = rand()*255;
        int g = rand()*255;
        int b = rand()*255;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(newCluster, r, g, b);
        viewer->addPointCloud<pcl::PointXYZ> (newCluster, single_color3, ost1.str());
    }
    while(!viewer->wasStopped()){
        viewer->spin();
    }
#endif
    return true;
}

bool ScrewDetector::getScrewCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointIndices::Ptr &otherinliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFilter){
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(otherinliersIds);
    //除去平面之外的数据
    extract.setNegative(true);
    extract.filter(*cloudFilter);
    std::cout<<otherinliersIds->indices.size()<<std::endl;

    return true;
}

void ScrewDetector::searchPixelInCloudMat(const cv::Mat & Img,  const pcl::PointXYZ & inputPt, cv::Point &outputPt, cv::Point knownPt , int  adjacent){
    if ( knownPt == cv::Point(0,0) && adjacent == 0 )
    {
        float minDis = sqrt(pow(Img.at<cv::Vec3f>(0,0)[0] - inputPt.x, 2 ) + pow(Img.at<cv::Vec3f>(0,0)[1] - inputPt.y, 2 ));
        for (int i = 0; i < Img.rows; i++)
            for (int j = 0; j < Img.cols; j++)
            {
                if(sqrt(pow(Img.at<cv::Vec3f>(i,j)[0] - inputPt.x, 2 ) + pow(Img.at<cv::Vec3f>(i,j)[1] - inputPt.y, 2 ))   <  minDis)
                {
                    minDis =sqrt(pow(Img.at<cv::Vec3f>(i,j)[0] - inputPt.x, 2 ) + pow(Img.at<cv::Vec3f>(i,j)[1] - inputPt.y, 2 ));
                    outputPt.x = j;
                    outputPt.y = i;
                }
            }
    }
    else{
        float minDis = sqrt(pow(Img.at<cv::Vec3f>(knownPt.y, knownPt.x)[0] - inputPt.x, 2 ) + pow(Img.at<cv::Vec3f>(knownPt.y, knownPt.x)[1] - inputPt.y, 2 ));
        for (int i = fmax(0, knownPt.y - adjacent); i < fmin(Img.rows, knownPt.y + adjacent); i++)
            for (int j = fmax(0, knownPt.x - adjacent); j< fmin(Img.cols, knownPt.x + adjacent); j++)
            {
                if (i > Img.rows || i < 0 || j > Img.cols || j < 0){
                    std::cout<<"search point _current (x,y):("<<j <<","<<i<<")"<<std::endl;
                }
                else{
                    if(sqrt(pow(Img.at<cv::Vec3f>(i,j)[0] - inputPt.x, 2 ) + pow(Img.at<cv::Vec3f>(i,j)[1] - inputPt.y, 2 )) < minDis)
                    {
                        minDis =sqrt(pow(Img.at<cv::Vec3f>(i,j)[0] - inputPt.x, 2 ) + pow(Img.at<cv::Vec3f>(i,j)[1] - inputPt.y, 2 ));
                        outputPt.x = j;
                        outputPt.y = i;
                    }
                }
            }
    }
}

bool ScrewDetector::getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters){
    std::vector<pcl::PointIndices> clusterIndices;
    //利用欧式聚类分割出各个点云簇，去除距离远的点，分离桌面与螺丝
    segmentEuclidean(cloud, _param.segScrewdistanceThreshold,_param.segScrewMinSize,_param.segScrewMaxSize,clusterIndices);
    for (int k = 0; k < clusterIndices.size(); ++k) {
        std::cout<<"clusterIndices["<<k<<"] ->indices=" <<clusterIndices[k].indices.size()<<std::endl;
    }

#ifdef DEBUG_SCREW
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("getClusters"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color0, "oricloud");
#endif
    for (int i = 0; i < clusterIndices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr screwcluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < clusterIndices[i].indices.size(); ++j) {
            screwcluster->points.push_back(cloud->points[clusterIndices[i].indices[j]]);
        }
        screwcluster->width = screwcluster->points.size();
        screwcluster->height = 1;
        screwcluster->is_dense = true;
        clusters.push_back(screwcluster);
#ifdef DEBUG_SCREW
        int r = rand()*255;
        int g = rand()*255;
        int b = rand()*255;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorin(screwcluster, r, g, b);
        std::string str;
        str = "cloud" + std::to_string(r+g+b);
        viewer->addPointCloud<pcl::PointXYZ> (screwcluster, single_colorin, str);
#endif
    }

#ifdef DEBUG_SCREW
    while(!viewer->wasStopped()) {
        viewer->spin();
    }
#endif
    return true;
}

struct BoundingBox {
    float scale;
    pcl::PointXYZ center;
    pcl::PointXYZ minPoint;
    pcl::PointXYZ maxPoint;
    pcl::PointXYZ position;
    pcl::PointXYZ xAxis;
    pcl::PointXYZ yAxis;
    pcl::PointXYZ zAxis;
    Eigen::Matrix3f rotationMatrix;
    Eigen::Vector3f majorVector;
    Eigen::Vector3f middleVector;
    Eigen::Vector3f minorVector;
};
void computeBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, BoundingBox &bbox) {
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> featureExtractor;
    featureExtractor.setInputCloud(cloud);
    featureExtractor.compute();

    std::vector<float> momentOfInertia;
    std::vector<float> eccentricity;
    float majorValue, middleValue, minorValue;
    Eigen::Vector3f massCenter;

    featureExtractor.getMomentOfInertia(momentOfInertia);
    featureExtractor.getEccentricity(eccentricity);
    featureExtractor.getOBB(bbox.minPoint, bbox.maxPoint, bbox.position, bbox.rotationMatrix);
    featureExtractor.getEigenValues(majorValue, middleValue, minorValue);
    featureExtractor.getEigenVectors(bbox.majorVector, bbox.middleVector, bbox.minorVector);
    featureExtractor.getMassCenter(massCenter);
    bbox.center.x = massCenter(0);
    bbox.center.y = massCenter(1);
    bbox.center.z = massCenter(2);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    bbox.scale = ((maxPt.z - minPt.z) + (maxPt.y - minPt.y) + (maxPt.x - minPt.x)) / 3;

    bbox.xAxis.x = bbox.scale * bbox.majorVector(0) + bbox.center.x;
    bbox.xAxis.y = bbox.scale * bbox.majorVector(1) + bbox.center.y;
    bbox.xAxis.z = bbox.scale * bbox.majorVector(2) + bbox.center.z;

    bbox.yAxis.x = bbox.scale * bbox.middleVector(0) + bbox.center.x;
    bbox.yAxis.y = bbox.scale * bbox.middleVector(1) + bbox.center.y;
    bbox.yAxis.z = bbox.scale * bbox.middleVector(2) + bbox.center.z;

    bbox.zAxis.x = bbox.scale * bbox.minorVector(0) + bbox.center.x;
    bbox.zAxis.y = bbox.scale * bbox.minorVector(1) + bbox.center.y;
    bbox.zAxis.z = bbox.scale * bbox.minorVector(2) + bbox.center.z;
}
bool ScrewDetector::getScrewPose(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, ObjectPose &objPoses) {
    std::vector<float> planeCoeff;
    int inlierSize = 0;
    pcl::PointIndices::Ptr inliersIds(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPt(new pcl::PointCloud<pcl::PointXYZ>);
    inliersIds = getmodelindices(cloud,coefficients,0,2);//平面模型获取螺丝平面内点及信息
    if(inliersIds->indices.size() < _param.segScrewMinSize){
        std::cout<<"screw cluster size too low"<<std::endl;
        return false;
    }
    //分割出平面点云
    inlierPt = extracindices(cloud,inliersIds,false);

    //1.计算各Cluster所在平面参数，并转换法向量方向
    if (coefficients->values[2] < 0){
        coefficients->values[0] *= -1;
        coefficients->values[1] *= -1;
        coefficients->values[2] *= -1;
        coefficients->values[3] *= -1;
    }
    planeCoeff.clear();
    planeCoeff.push_back(coefficients->values[0]);
    planeCoeff.push_back(coefficients->values[1]);
    planeCoeff.push_back(coefficients->values[2]);
    planeCoeff.push_back(coefficients->values[3]);

    pcl::Normal objNormal(planeCoeff[0], planeCoeff[1], planeCoeff[2]);

    //2.判断法向量与Z方向夹角，认为法向与Z轴夹角小于15-20度（待测试）为正常螺丝
    Eigen::Vector3d planeNormal(0, 0, 0);//平面法向方向
    Eigen::Vector3d nz(0, 0, 1);//平面法向方向

    planeNormal << planeCoeff[0], planeCoeff[1], planeCoeff[2];

    Eigen::Vector3d bkgroundNormal(bkgroundPlaneCoeff[0],bkgroundPlaneCoeff[1],bkgroundPlaneCoeff[2]);
    float theta = acos(planeNormal.dot(bkgroundNormal) /(bkgroundNormal.norm()*  planeNormal.norm())) * 180.0 / CV_PI;
    if (theta > _param.maxAngleInDegreeWithBkground) {
        std::cout<<"theta = "<<theta<<std::endl;
        return false;
    }

    //3.旋转点云至水平，求内点BoundingBox，认为螺丝长宽比例大于0.85(待测试)，现采集螺丝点云，边缘部分点云基本保留完整
    for (int j = 0; j < inliersIds->indices.size(); ++j) {
        inlierPt->points.push_back(cloud->points[inliersIds->indices[j]]);
    }
    //3.1旋转点云至水平
    Eigen::Vector3d cNormal;
    cNormal = planeNormal.cross(nz);

    Eigen::AngleAxisd rotationVector(acos(planeNormal.dot(nz)), cNormal);
    Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
    rotationMatrix = rotationVector.toRotationMatrix();

    Eigen::Matrix4d transferM;
    transferM.block(0, 0, 3, 3) = rotationMatrix;
    transferM.row(3) << 0, 0, 0, 1;
    transferM.col(3) << 0, 0, 0, 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPoint(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformedPoint, transferM);

    //3.2计算BoundingBox，判断
    BoundingBox bdBox;
    computeBoundingBox(transformedPoint,bdBox);

    float Length = std::min((bdBox.maxPoint.x - bdBox.minPoint.x),(bdBox.maxPoint.y - bdBox.minPoint.y));
    float Width = std::max((bdBox.maxPoint.x - bdBox.minPoint.x),(bdBox.maxPoint.y - bdBox.minPoint.y));
    float ratioLength2Width =  Length/Width;//确保ratio <= 1
    std::cout<<"ratio = "<<ratioLength2Width<<std::endl;
    if(ratioLength2Width < 0.8)return false;

    Eigen::Vector4d center;
    center << bdBox.position.x, bdBox.position.y, bdBox.position.z + bdBox.minPoint.z, 1;
    Eigen::Vector4d tCenter;
    tCenter = transferM.inverse() * center;//螺丝上表面中心位置
    std::cout<<"tCenter = "<<tCenter<<std::endl;

#ifdef DEBUG_SCREW
    //4.判断螺丝点云的周围点云，若周围点云平均比螺丝点云低，认为是螺丝
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("getScrewPose"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
    viewer2->addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud");

    while(!viewer2->wasStopped()){
        viewer2->spin();
    }
#endif

    pcl::PointXYZ tmpcenter;
    tmpcenter.x = tCenter[0];
    tmpcenter.y = tCenter[1];
    tmpcenter.z = tCenter[2];

    objPoses.normal = objNormal;

    objPoses.center = tmpcenter;

    return true;
}

//  SAC 分割，设置不同的模型，得到与模型类似的点云 输出拟合模型点云索引
pcl::PointIndices::Ptr ScrewDetector::getmodelindices(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                      pcl::ModelCoefficients::Ptr &coefficients,
                                                      int modeltype = -1,
                                                      double  distanceThred = -1 ){
    double time0 = cv::getTickCount();
    /*
     * SAC分割，根据不同的模型可以得到符合近似模型要求的点云
     * 可以先从整体以平面，圆柱，棒，圆锥，３Ｄ圆，２D圆等模型形式分割出目标可能存在的点云区域
     * 再将点云欧式聚类分割或使用不同的模型多次分割
     */
//        std::cout<<"modeltype : "<<modeltype<<std::endl;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);          // 同时优化模型系数
    //  模型类型设置，可以在函数中直接设置，也可以在参数文件中设置
    if (modeltype == -1) {
        switch (_param.SACmodel){
            case 0:
                seg.setModelType (pcl::SACMODEL_PLANE);         // 平面模型
                break;
            case 1:
                seg.setModelType (pcl::SACMODEL_STICK);         // 棒模型
                break;
            case 2:
                seg.setModelType (pcl::SACMODEL_CIRCLE2D);      // 2D圆模型
                break;
            case 3:
                seg.setModelType (pcl::SACMODEL_CIRCLE3D);      // 3D圆模型
                break;
            case 4:
                seg.setModelType (pcl::SACMODEL_CYLINDER);      // 圆柱模型
                break;
            case 5:
                seg.setModelType (pcl::SACMODEL_CONE);          // 圆锥模型
                break;
            case 6:
                seg.setModelType (pcl::SACMODEL_SPHERE);        // 球模型
                break;
            default:
                std::cerr << "Din't setModelType, pclease chechk! \n";
        }

    } else {
        switch (modeltype){
            case 0:
                seg.setModelType (pcl::SACMODEL_PLANE);         // 平面模型
                break;
            case 1:
                seg.setModelType (pcl::SACMODEL_STICK);         // 棒模型
                break;
            case 2:
                seg.setModelType (pcl::SACMODEL_CIRCLE2D);      // 2D圆模型
                break;
            case 3:
                seg.setModelType (pcl::SACMODEL_CIRCLE3D);      // 3D圆模型
                break;
            case 4:
                seg.setModelType (pcl::SACMODEL_CYLINDER);      // 圆柱模型
                break;
            case 5:
                seg.setModelType (pcl::SACMODEL_CONE);          // 圆锥模型
                break;
            case 6:
                seg.setModelType (pcl::SACMODEL_SPHERE);        // 球模型
                break;
            default:
                std::cerr << "Din't setModelType, pclease chechk! \n";
        }
    }
    //  ＳＡＣ分割距离阈值，可以在函数中直接设置，也可以在参数文件中设置
    if (distanceThred == -1) {
        seg.setDistanceThreshold(_param.SACdistanceThreshold);          // 距离阈值
    } else
    {
        seg.setDistanceThreshold(distanceThred);                        // 距离阈值
    }

//        std::cout<<"Model typer : "<<seg.getModel()<<std::endl;
//        std::cout<<"Distance Threshold : "<<seg.getDistanceThreshold()<<std::endl;
//        seg.setMaxIterations (_param.SACmaxIterations);                // 迭代次数
    seg.setMethodType (pcl::SAC_RANSAC);       // 随机采样一致方法
//        seg.setNormalDistanceWeight (_param.SACnormalweight);           // 法线信息权重
//        seg.setRadiusLimits (0, 5);                 // 半径区间
//        seg.setSamplesMaxDist (20,tree);             // 采样最大距离.
    seg.setProbability (0.8);                   // 采样最大距离

    seg.setInputCloud (cloud);

    double time1 = cv::getTickCount();

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    std::cout<<"cloud->points.size() = "<<cloud->points.size()<<std::endl;
    normals = getCloudNormal(cloud);
    std::cout<<"normals finish";
    double time2 = cv::getTickCount();

    seg.setInputNormals (normals);
    std::cout<<"seg.setInputNormals finish";

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.segment (*inliers , *coefficients );
    std::cout<<"seg.segment finish";
    double time3 = cv::getTickCount();

    std::cout<<"set param cost time = "<<1000*(time1-time0)/cv::getTickFrequency()<<std::endl;
    std::cout<<"get normals cost time = "<<1000*(time2-time1)/cv::getTickFrequency()<<std::endl;
    std::cout<<"segment cost time = "<<1000*(time3-time2)/cv::getTickFrequency()<<std::endl;
//    std::cerr << "Model coefficients: " << *coefficients << std::endl;
//        std::cerr << "Mpdel inliers: " << inliers->indices.size() << std::endl;
    return inliers;
}

pcl::PointCloud<pcl::Normal>::Ptr ScrewDetector::getCloudNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr const &cloud ) {
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//    std::cout<<"use ne.threads_ num = "<<ne.threads_<<std::endl;
//        ne.setNumberOfThreads(2);
//        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    std::cout<<"_param.normalRadius = "<<_param.normalRadius<<std::endl;
    ne.setKSearch (_param.normalRadius);
    ne.compute (*normals);
    return normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ScrewDetector::extracindices (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointIndices::Ptr inliers, bool negative = false) {
    // Extract the inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    if (negative == true){
        extract.setNegative (true);
    } else{extract.setNegative(false);}
    pcl::PointCloud<pcl::PointXYZ>::Ptr modelcloud (new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter (*modelcloud);
//    std::cout << "Model: " << modelcloud->points.size() << std::endl;
    return modelcloud;
}

//传入2D彩色图，并利用计算得到的maskImg为螺丝边缘画线，显示螺丝在图中的位置
bool ScrewDetector::getImgwithMask(cv::Mat &inImg,cv::Mat &inmaskImg,cv::Mat &outImg){
    if (inImg.channels() != 3){
        std::cout<<"请检查传入图像，是否为彩色图"<<std::endl;
        return false;
    }
    outImg = inImg.clone();
    double min, max;
    cv::minMaxLoc(inmaskImg, &min, &max);
    std::cout<<max<<std::endl;

    if (max < 1) {
        std::cout << "mask中无有效标记" << std::endl;
        return false;
    }else{
        for (int i = 1; i <= max; ++i) {
            cv::Mat singleMask;
            cv::compare(inmaskImg, i, singleMask, cv::CMP_EQ);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(singleMask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);

            std::vector<int> hull;
            cv::convexHull(cv::Mat(contours[0]),hull,false);//找离散点的边界
            cv::Point root_points[1][hull.size()];
            for (int i = 0; i < hull.size(); ++i) {
                root_points[0][i] = contours[0][hull[i]];
            }

            const cv::Point* pt[1] = {root_points[0]};
            int npt[] = {(int)hull.size()};
            cv::polylines(outImg, pt, npt,1, true, cv::Scalar(0,0,255),5);
        }
    }
    return true;
}
//2.利用变换矩阵对2D图像进行变换，获取与3D相机的2D图像相匹配的2D相机的2D图像
/*
 * inImg为变换前的图像，为2D相机获取的图像
 * outImg为变换后的图像，为2D相机按照矩阵变换后得到的图像 是与3D相机的2D图像配准后的图像
 * outImg也是用于在界面上显示的图像,仅用于显示
 * */
bool ScrewDetector::getColor2DImg(cv::Mat &inImg ,int outputHeight,int outputWidth ,cv::Mat &outImg){
    bool res = transform2DImg(inImg ,outImg , outputHeight , outputWidth, _param.Mat_2DCamera23DCamera,0);
    if (!res){
        return false;
    }
    return true;
}

//根据2D、3D相机标定矩阵
bool ScrewDetector::transform2DImg(cv::Mat &inImg ,cv::Mat &outImg, int outputHeight,int outputWidth ,cv::Mat &T,int T_Type){
    switch (T_Type){
        case 0:
            if (T.rows != 3 || T.cols !=3){
                return false;
            }
            std::cout<<T<<std::endl;
            cv::warpPerspective(inImg,outImg, T ,cv::Size(outputWidth,outputHeight));
        default:
            break;
    }
    return true;
}
//根据传入图像坐标点，查找返回当前点击区域对应的mask值
bool ScrewDetector::getScrewMaskNum(cv::Point &inpt2d,int &num){
    num = _maskImg.at<uchar>(inpt2d.y,inpt2d.x);
    if (num == 0){
        return false;
    }
    return true;
}
///////////////////////////////////////////////
bool ScrewDetector::setParam(const std::string &fileFullName) {
    detectParam param;
    if (loadParamFile(fileFullName, param)) {
        initParam(param);
        return true;
    } else {
        return false;
    }
}

void ScrewDetector::initParam(const detectParam &param) {
    this->_param.roiX = param.roiX;
    this->_param.roiY = param.roiY;
    this->_param.roiW = param.roiW;
    this->_param.roiH = param.roiH;

    this->_param.ptMinZ = param.ptMinZ;
    this->_param.ptMaxZ = param.ptMaxZ;
    this->_param.ptMinX = param.ptMinX;
    this->_param.ptMaxX = param.ptMaxX;
    this->_param.ptMinY = param.ptMinY;
    this->_param.ptMaxY = param.ptMaxY;

    this->_param.downSampeX = param.downSampeX;
    this->_param.downSampeY = param.downSampeY;
    this->_param.downSampeZ = param.downSampeZ;

    this->_param.meanK = param.meanK;
    this->_param.mulStd = param.mulStd;

    this->_param.filterRadius = param.filterRadius;
    this->_param.filterMinNum = param.filterMinNum;

    this->_param.segdistanceThreshold = param.segdistanceThreshold;
    this->_param.segBkgroundMinSize = param.segBkgroundMinSize;
    this->_param.segBkgroundMaxSize = param.segBkgroundMaxSize;

    this->_param.normalRadius = param.normalRadius;

    this->_param.maxAngleInDegreeWithBkground = param.maxAngleInDegreeWithBkground;

    this->_param.segScrewdistanceThreshold = param.segScrewdistanceThreshold;
    this->_param.segScrewMinSize = param.segScrewMinSize;
    this->_param.segScrewMaxSize = param.segScrewMaxSize;


    this->_param.Mat_2DCamera23DCamera = param.Mat_2DCamera23DCamera;
//
//        //重建参数
//        //贪婪三角化参数
//        this->_param.greedyConnectPointDisMax = param.greedyConnectPointDisMax ;
//        this->_param.greedySearchDisMax = param.greedySearchDisMax ;
//        this->_param.greedyNearestNeighborsNum = param.greedyNearestNeighborsNum;
//        this->_param.greedySurfaceAngle = param.greedySurfaceAngle;
//        this->_param.greedyMinInteriorAngle = param.greedyMinInteriorAngle;
//        this->_param.greedyMaxInteriorAngle = param.greedyMaxInteriorAngle;
//        this->_param.greedyNormalConsistency = param.greedyNormalConsistency;
//        //泊松表面重建Poisson
//        this->_param.poissonConfidence = param.poissonConfidence;
//        this->_param.poissonDegree = param.poissonDegree;
//        this->_param.poissonDepth = param.poissonDepth;
//        this->_param.poissonIsoDivide = param.poissonIsoDivide;
//        this->_param.poissonManifold = param.poissonManifold;
//        this->_param.poissonOutputPolygons = param.poissonOutputPolygons;
////        this->_param.poissonSamplesPerNode = param.poissonSamplesPerNode;
//        this->_param.poissonScale = param.poissonScale;
//        this->_param.poissonSolverDivide = param.poissonSolverDivide;
//
//
//
//        this->_param.segCurvatureThreshold = param.segCurvatureThreshold;
//        this->_param.segMaxSize = param.segMaxSize;
//        this->_param.segMinSize = param.segMinSize;
//        this->_param.segNeighborNumber = param.segNeighborNumber;
//        this->_param.segResidualThreshold = param.segResidualThreshold;
//        this->_param.segThetaThresholdInDegree = param.segThetaThresholdInDegree;
//        this->_param.segMaxDis = param.segMaxDis;
//
//
//
//        this->_param.distanceThreshold = param.distanceThreshold;
//        this->_param.maxIterations = param.maxIterations;
//        this->_param.planeInliersRatio = param.planeInliersRatio;
//
//        this->_param.maxAngleInDegreeWithZ = param.maxAngleInDegreeWithZ;
//        this->_param.widthThreshold = param.widthThreshold;
//        this->_param.heightThreshold = param.heightThreshold;
}

bool ScrewDetector::loadParamFile(const std::string &fileFullName, detectParam &param) {
     cv::FileStorage fs(fileFullName, cv::FileStorage::READ);
    if (fs.isOpened()) {
        std::string s;
        param.roiX = fs["roiX"];
        param.roiY = fs["roiY"];
        param.roiW = fs["roiW"];
        param.roiH = fs["roiH"];
        param.ptMinZ = fs["ptMinZ"];
        param.ptMaxZ = fs["ptMaxZ"];
        param.ptMinX = fs["ptMinX"];
        param.ptMaxX = fs["ptMaxX"];
        param.ptMinY = fs["ptMinY"];
        param.ptMaxY = fs["ptMaxY"];
        param.downSampeX = fs["downSampeX"];
        param.downSampeY = fs["downSampeY"];
        param.downSampeZ = fs["downSampeZ"];
        param.meanK = fs["meanK"];
        param.mulStd = fs["mulStd"];
        param.segdistanceThreshold = fs["segdistanceThreshold"];
        param.segBkgroundMinSize = fs["segBkgroundMinSize"];
        param.segBkgroundMaxSize = fs["segBkgroundMaxSize"];
        param.maxAngleInDegreeWithBkground = fs["maxAngleInDegreeWithBkground"];
        param.segScrewdistanceThreshold = fs["segScrewdistanceThreshold"];
        param.segScrewMinSize = fs["segScrewMinSize"];
        param.segScrewMaxSize = fs["segScrewMaxSize"];


        cv::Mat frontCenter2Zero_Mat(3,3,CV_32F);
        fs["Mat_2DCamera23DCamera"]>>param.Mat_2DCamera23DCamera;

//            //三维重建
//            param.greedyConnectPointDisMax = fs["greedyConnectPointDisMax"];//贪婪三角化连接点之间最大距离
//            param.greedySearchDisMax = fs["greedySearchDisMax"];//设置连接点最大距离
//            param.greedyNearestNeighborsNum = fs["greedyNearestNeighborsNum"];//设置可搜索邻域个数
//            param.greedySurfaceAngle =fs["greedySurfaceAngle"];//设置法线方向偏离样本的最大角度
//            param.greedyMinInteriorAngle = fs["greedyMinInteriorAngle"];//设置三角化得到的内角最小角度
//            param.greedyMaxInteriorAngle = fs["greedyMaxInteriorAngle"];//设置三角化得到的内角最小角度
//            s = (std::string)fs["greedyNormalConsistency"];//设置保证法线朝向一致
//            std::istringstream(s)>>param.greedyNormalConsistency;
//
//            //泊松表面重建Poisson
//            s = (std::string)fs["poissonConfidence"];
//            std::istringstream(s)>>param.poissonConfidence;
//            param.poissonDegree = fs["poissonDegree"];
//            param.poissonDepth = fs["poissonDepth"];
//            param.poissonIsoDivide = fs["poissonIsoDivide"];
//            s = (std::string)fs["poissonManifold"];
//            std::istringstream(s)>>param.poissonManifold;
//            s = (std::string)fs["poissonOutputPolygons"];
//            std::istringstream(s)>>param.poissonOutputPolygons;
////            param.poissonSamplesPerNode = fs["poissonSamplesPerNode"];
//            param.poissonScale = fs["poissonScale"];
//            param.poissonSolverDivide = fs["poissonSolverDivide"];
        param.filterRadius = fs["filterRadius"];
        param.filterMinNum = fs["filterMinNum"];
        param.normalRadius = fs["normalRadius"];
        param.segMinSize = fs["segMinSize"];
        param.segMaxSize = fs["segMaxSize"];
        param.segResidualThreshold = fs["segResidualThreshold"];
        param.segCurvatureThreshold = fs["segCurvatureThreshold"];
        param.segThetaThresholdInDegree = fs["segThetaThresholdInDegree"];
        param.segNeighborNumber = fs["segNeighborNumber"];
        param.segMaxDis = fs["segMaxDis"];
        param.distanceThreshold = fs["distanceThreshold"];
        param.maxIterations = fs["maxIterations"];
        param.planeInliersRatio = fs["planeInliersRatio"];
        param.maxAngleInDegreeWithZ = fs["maxAngleInDegreeWithZ"];
        param.widthThreshold = fs["widthThreshold"];
        param.heightThreshold = fs["heightThreshold"];
        return true;
    } else {
        return false;
    }
}

void ScrewDetector::generatePointCloud(const std::string &path_xy, const std::string &path_pointcloud, cv::Mat &pointCloud) {
    std::ifstream coordinate_xys;
    std::vector<cv::Point> coordinate_xy;
    cv::Point coordinate_temp;
    int coordinate_temp_x = 0;
    int coordinate_temp_y = 0;
    coordinate_xys.open(path_xy);
    assert(coordinate_xys.is_open());
    while (coordinate_xys >> coordinate_temp_x >> coordinate_temp_y) {
        coordinate_temp.x = coordinate_temp_x;
        coordinate_temp.y = coordinate_temp_y;
        coordinate_xy.push_back(coordinate_temp);
    }
//        std::cout << coordinate_xy.size() << std::endl;

    std::ifstream pointcloud_xyzs;
    std::vector<cv::Point3f> pointcloud_xyz;
    cv::Point3f pointcloud_temp;
    float pointcloud_temp_x = 0.0;
    float pointcloud_temp_y = 0.0;
    float pointcloud_temp_z = 0.0;
    pointcloud_xyzs.open(path_pointcloud);
    assert(pointcloud_xyzs.is_open());
    while (pointcloud_xyzs >> pointcloud_temp_x >> pointcloud_temp_y >> pointcloud_temp_z) {
        pointcloud_temp.x = pointcloud_temp_x;
        pointcloud_temp.y = pointcloud_temp_y;
        pointcloud_temp.z = pointcloud_temp_z;
        pointcloud_xyz.push_back(pointcloud_temp);
    }
    std::cout << "original num of Points11: " << pointcloud_xyz.size() << std::endl;
//        pointCloud.at<cv::Vec3f>(2047,2447)[0] = 100;
    int k = 0;
    for (auto p : coordinate_xy) {
        pointCloud.at<cv::Vec3f>(2048 - p.x, p.y)[0] = pointcloud_xyz[k].x;
        pointCloud.at<cv::Vec3f>(2048 - p.x, p.y)[1] = pointcloud_xyz[k].y;
        pointCloud.at<cv::Vec3f>(2048 - p.x, p.y)[2] = pointcloud_xyz[k].z;
        ++k;
    }
    std::cout << "original num of Points: " << pointcloud_xyz.size() << std::endl;
}

void ScrewDetector::matToPCD(const cv::Mat &pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool bSave) {
    //转化成点云格式
    cloud->clear();
    for (unsigned int i = 0; i < pointCloud.rows; i++) {
        for (unsigned int j = 0; j < pointCloud.cols; j++) {
            if (pointCloud.at<cv::Vec3f>(i, j)[2] < 1) {
                continue;
            } else {
                pcl::PointXYZ point;
                point.x = pointCloud.at<cv::Vec3f>(i, j)[0];
                point.y = pointCloud.at<cv::Vec3f>(i, j)[1];
                point.z = pointCloud.at<cv::Vec3f>(i, j)[2];
                cloud->points.push_back(point);
            }
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    std::cout << "cloud size " << cloud->points.size() << std::endl;
    if (bSave) {
        pcl::io::loadPLYFile("totalCloud.pcd", *cloud);
    }
}

bool ScrewDetector::cropROI(const cv::Mat &imgOrg, const cv::Mat &pcOrg, cv::Mat &imgROI, cv::Mat &pcROI) {
    imgROI = imgOrg(cv::Rect(_param.roiX, _param.roiY, _param.roiW, _param.roiH)).clone();
    pcROI = pcOrg(cv::Rect(_param.roiX, _param.roiY, _param.roiW, _param.roiH)).clone();
#ifdef DEBUG_MEDICINE_BOX
    cv::namedWindow("roiImg", CV_WINDOW_NORMAL);
    cv::imshow("roiImg", imgROI);
    cv::namedWindow("roiPt", CV_WINDOW_NORMAL);
    cv::imshow("roiPt", pcROI);
    cv::waitKey(1);
#endif
    return true;
}
