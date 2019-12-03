/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-10-31         zhangxiaohan
============================================================== **/
#include <LocalStorageRepositoryFactoryInterface.h>
#include <RobotDriverFactoryInterface.h>
#include <logger/Logger.h>
#include <RobotStatusEnum.h>
#include <ArmRobotWayPointStruct.h>
#include <ArmRobotMotionTypeEnum.h>
#include <LocalStorageRepositoryAPIKey.h>

using namespace LION_LSR_NS_API;
using namespace WOLF_ROBOTDRIVER_NS_API;

int main(int argc, char** argv)
{
    //从lsr 获取机器人名
    boost::shared_ptr<LocalStorageRepositoryFactoryInterface> lsrFactory = LocalStorageRepositoryFactoryInterface::create();
    boost::shared_ptr<LocalStorageRepositoryInterface> lsr = lsrFactory->createLocalStorageRepository();
    std::string robotName = lsr->getItem(app_test_robot_name);
    std::string ip = lsr->getItem(app_test_robot_ip);
    LOG_INFO << "Robot name=" << robotName << " ip=" << ip;

    //创建机器人对象
    boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
    boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver(robotName);

    //连接机器人
    RobotStatus status = robotDriver->connect(ip);
    LOG_INFO << "robot connect Status:" << RobotStatusEnum2String(status);

    //让机器人画三角形
    std::vector<ArmRobotWayPoint> movePath;
    ArmRobotWayPoint temp;

    //获取机器人当前joint值
    std::vector<std::vector<double>> joints;
    std::vector<double> joint1 = {1.57,-1.67,1.68,-1.59,-1.57,-0.76};
    std::vector<double> joint2 = {1.57066,-2.35917,2.0515,-1.27281,-1.56982,0.810949};
    std::vector<double> joint3 = {2.36336,-1.89956,1.85443,-1.53307,-1.577,1.60368};
    joints.push_back(joint1);
    joints.push_back(joint2);
    joints.push_back(joint3);

    int loopTimes = 10;
    if (argc > 1) {
        loopTimes = atoi(argv[1]);
    }
    for (int i = 0; i < loopTimes; i++) {
        for(size_t i=0;i<joints.size();i++)
        {
            temp.jointPosition = joints[i];
            temp.motionType = ArmRobotMotionType::MoveL;
            //速度和加速度都为百分比
            temp.acc = 10.00;
            temp.vel = 20.00;
            temp.blendRaidus = 0.00;
            movePath.push_back(temp);
            LOG_INFO << "moveArm result=" << robotDriver->moveArm(movePath) << "[" << i << "]";
            movePath.clear();
        }
    }
    return 0;
}

