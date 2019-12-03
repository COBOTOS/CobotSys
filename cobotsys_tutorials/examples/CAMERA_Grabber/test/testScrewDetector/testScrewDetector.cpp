#include <iostream>

#include "ScrewDetector.h"
#include <pcl/common/common_headers.h>

#define GET_SCREWPOSE

template <typename  PointT>
bool showcloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer_window,std::string strviewer ,typename pcl::PointCloud<PointT>::Ptr &clouds,uchar r,uchar g,uchar b){
    //    /*图形显示模块*/
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_cloud(clouds, r,g,b);
    viewer_window->addPointCloud(clouds,single_color_cloud,strviewer);
    return true;
}

//构造球体点云
void ConsSphereCloud(float center_x,float center_y,float center_z,float r,pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr)
{
    float radius = r;

    for (float angle1 = 0.0; angle1 <= 180.0; angle1 += 5.0)
    {
        for (float angle2 = 0.0; angle2 <= 360.0; angle2 += 5.0)
        {
            pcl::PointXYZ basic_point;

            basic_point.x = center_x + radius * sinf(pcl::deg2rad(angle1)) * cosf(pcl::deg2rad(angle2));
            basic_point.y = center_y + radius * sinf(pcl::deg2rad(angle1)) * sinf(pcl::deg2rad(angle2));
            basic_point.z = center_z + radius * cosf(pcl::deg2rad(angle1));
            basic_cloud_ptr->points.push_back(basic_point);
        }
    }
    basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1;
    basic_cloud_ptr->is_dense = true;
}

int main(){

#ifdef GET_SCREWPOSE

    //加载点云
    cv::Mat inimg;
    cv::FileStorage fr("../test/testScrewDetector/mat.pcd", cv::FileStorage::READ);
    fr["data"]>>inimg;
    fr.release();

    //获取参数
    std::string paramFullPath = "../test/testScrewDetector/screw_para.xml";
    std::shared_ptr<ScrewDetector> screw = std::make_shared<ScrewDetector>(paramFullPath);

    pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>);
    double time01 = cv::getTickCount();
    screw->getptcloudformmat(inimg,incloud);

    screw->processVisionImage(inimg);
    std::vector<Eigen::Matrix4d> screw_poses = screw->getScrewPose();
    int num =0;

    //*图形显示模块*//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_all(new pcl::visualization::PCLVisualizer("3D viewer_screw"));
    showcloud<pcl::PointXYZ>(viewer_all,"incloud",incloud,255,255,255);
    for (auto i : screw_poses) {

        std::cout<<"screw_poses:"<<i<<std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr screw_000(new pcl::PointCloud<pcl::PointXYZ>);

        ConsSphereCloud(i(0,3),i(1,3),i(2,3),4,screw_000);
        uchar r = 255*rand();
        uchar g = 255*rand();
        uchar b = 255*rand();

        showcloud<pcl::PointXYZ>(viewer_all,"screw_poses"+std::to_string(num),screw_000,r,g,b);
        num++;
    }

    while(!viewer_all->wasStopped()){
        viewer_all->spin();
    }


    return 0;
#endif




}
