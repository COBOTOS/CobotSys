/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-11-12       cobot          Initial Draft
============================================================== **/

#include <Camera3DFactoryInterface.h>
#include <opencv2/opencv.hpp>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
//统一的日志管理,使用此头文件
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>


//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;
using namespace EAGLE_DATA_NS_API;

int main()
{
    //使用日志配置时使用,必须要有配置文件
//    glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
    //创建相机工厂对象
    shared_ptr<Camera3DFactoryInterface> Factory = Camera3DFactoryInterface::create();
    //通过工厂对象创建对象
    shared_ptr<Camera3DInterface> camera = Factory->createCamera3D("RealSense");
    int ret1 = camera->open("");
    LOG_INFO << "connect camera result=" << ret1;

    for(int i = 0 ;i < 100000;i++){
        std::vector<boost::shared_ptr<VisionInputImage>> images = camera->captureSync();
        cv::namedWindow("2D",cv::WINDOW_AUTOSIZE);
        cv::imshow("2D",*images[0]->image);
        cv::namedWindow("3D",cv::WINDOW_AUTOSIZE);
        cv::imshow("3D",*images[1]->image);
//        cv::namedWindow("cloud",cv::WINDOW_AUTOSIZE);
//        cv::imshow("cloud",*images[2]->image);
        cv::waitKey(10);
    }
    const char*path = "/home/cobot/Desktop/test.bmp";
//    cv::imwrite(path,*images[1]->image);
//    LOG_INFO << "Pic count" << images.size();

    camera->close();
}
