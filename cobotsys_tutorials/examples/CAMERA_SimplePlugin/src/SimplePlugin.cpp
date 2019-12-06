//
// Created by 杨帆 on 17-7-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "SimplePlugin.h"
#include <RobWorkStudio.hpp>
#include <logger/Logger.h>
#include "ui_SimplePlugin.h"

#include <Camera3DFactoryInterface.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
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

SimplePlugin::SimplePlugin() :
        rws::RobWorkStudioPlugin(tr("SimplePlugin"),QIcon("SP")) {
    initWidget(this);
    LOG_INFO << "construct RealsensePlugin";
}

SimplePlugin::~SimplePlugin() {
}

void SimplePlugin::initWidget(QDockWidget *parent) {
    ui = new Ui::SimplePluginUI();
    ui->setupUi(parent);

    auto _cplusThread = std::make_shared<std::thread>(&SimplePlugin::test,this);
    _cplusThread->detach();
}

void SimplePlugin::close(){

}

void SimplePlugin::test(){
    //创建相机工厂对象
    shared_ptr<Camera3DFactoryInterface> Factory = Camera3DFactoryInterface::create();
    //通过工厂对象创建对象
    shared_ptr<Camera3DInterface> camera = Factory->createCamera3D("RealSense");
    int ret1 = camera->open("");
    LOG_INFO << "connect camera result=" << ret1;

    for(int i = 0 ;i < 100000;i++){
        std::vector<boost::shared_ptr<VisionInputImage>> images = camera->captureSync();
        auto tmpMat = *images[0]->image;

        auto cvWidth=tmpMat.cols;
        auto cvHeight=tmpMat.rows;
        if (tmpMat.channels() == 1) {
            cv::cvtColor(tmpMat, tmpMat, CV_BGR2GRAY);
        } else {
            cv::cvtColor(tmpMat, tmpMat, CV_BGR2RGB);
        }
        QImage tmpQImage = QImage((const uchar *) (tmpMat.data), tmpMat.cols, tmpMat.rows, tmpMat.step,
                               QImage::Format_RGB888);

        ui->label->setPixmap(QPixmap::fromImage(tmpQImage).scaled(cvWidth, cvHeight));
    }
    camera->close();
}