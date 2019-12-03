/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-11-12       cobot          Initial Draft
============================================================== **/
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h
#include <ForceSensorFactoryInterface.h>
//创建力传感器接口对象时,使用此头文件ForceSensorInterface.h
#include <ForceSensorInterface.h>
#include <LocalStorageRepositoryAPIKey.h>
#include <boost/shared_ptr.hpp>
//统一的日志管理,使用此头文件
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
//#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;

int main()
{
    //使用日志配置时使用,必须要有配置文件
//    glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
    //创建力传感器工厂对象
    boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
    //通过力传感器工厂对象创建力传感器对象
    boost::shared_ptr<ForceSensorInterface> forceSensor = forceSensorFactory->createForceSensor("OptoForceSensor");
    //设置 SensorInfo
    SensorInfo sensorInfo;
    sensorInfo.uri = "192.168.1.1";
    ForceSensorStatus forceSensorStatus;
    //连接ForceSensor
    forceSensorStatus = forceSensor->connect(sensorInfo);
    LOG_INFO << "connect forceSensor result = " << ForceSensorStatusEnum2String(forceSensorStatus);
    //获取ForceSensor状态
    forceSensorStatus = forceSensor->getSensorStatus();
    LOG_INFO << "forceSensor status = " << ForceSensorStatusEnum2String(forceSensorStatus);
    while(1)
    {
        //获取Data数据
        std::vector<double> forceSensorDatas = forceSensor->getSensorData();
        LOG_INFO << "forceSensorDatas.size() = " << forceSensorDatas.size() << "; forceSensorDatas = " << forceSensorDatas;
//        计算内部误差
//        bool ret2 = forceSensor->calculateIntrinsicError();
//        if(ret2){
//            LOG_INFO << "Calculate Intrinsic Error successfully!";
//        }
//        获取接触力
//        std::vector<double> contactForce = forceSensor->getContactForce();
//        LOG_INFO << "contactForce:" << contactForce;
    }
}
