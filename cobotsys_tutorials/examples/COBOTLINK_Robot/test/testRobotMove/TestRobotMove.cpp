/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-10-30       zhoupeng      Initial Draft
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

    //让机器人绕着基座标旋转
    std::vector<ArmRobotWayPoint> movePath;
    ArmRobotWayPoint temp;
    //自己定义一个常数
    double turnPi = 3.141592654/180;

    //获取机器人当前joint值
    std::vector<double> joint1 = robotDriver->getRobotJoints();
    int loopTimes = 10;
    if (argc > 1) {
        loopTimes = atoi(argv[1]);
    }
    for (int i = 0; i < loopTimes; i++) {
        temp.jointPosition = joint1;
        temp.motionType = ArmRobotMotionType::MoveJ;
        //速度和加速度都为百分比
        temp.acc = 10.00;
        temp.vel = 20.00;
        temp.blendRaidus = 0.00;
        movePath.push_back(temp);
        LOG_INFO << "moveArm result=" << robotDriver->moveArm(movePath) << "[" << i << "]";
        movePath.clear();
        joint1[0] += 5*turnPi;
    }
    return 0;
}