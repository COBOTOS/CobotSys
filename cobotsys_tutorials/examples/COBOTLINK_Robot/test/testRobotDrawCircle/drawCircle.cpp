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
#include <KinematicSolverFactoryInterface.h>
#include <KinematicSolverInterface.h>
#include <ArmRobotWayPointStruct.h>
#include <ArmRobotMotionTypeEnum.h>
#include <LocalStorageRepositoryAPIKey.h>

using namespace LION_LSR_NS_API;
using namespace WOLF_ROBOTDRIVER_NS_API;
using namespace WOLF_KINEMATICSOLVER_NS_API;

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

    //创建运动学解析器
    boost::shared_ptr<KinematicSolverFactoryInterface> kinematicSolverFactory = KinematicSolverFactoryInterface::create();
    boost::shared_ptr<KinematicSolverInterface> kinematicSolver = kinematicSolverFactory->createKinematicSolver(robotName);

    //连接机器人
    RobotStatus status = robotDriver->connect(ip);
    LOG_INFO << "robot connect Status:" << RobotStatusEnum2String(status);

    //让机器人画圆
    std::vector<ArmRobotWayPoint> movePath;
    ArmRobotWayPoint temp;
    double turnPI = 3.141592654/180;
    //获取机器人当前joint值
    std::vector<double> initialJoint = {0*turnPI,-90*turnPI,-90*turnPI,-90*turnPI,90*turnPI,0*turnPI};
    std::vector<double> targetJoint(6,0);
    std::vector<double> targetPos = robotDriver->getRobotTcp();
    LOG_INFO << "Robot TCP = " << targetPos;

    double X = 0;
    double Y = 0;
    double Z = 0.5;
    double r = 150;//半径
//重复画两次圆
    for(int i=0;i<720;i++) {
        X = (164 + cos(i * turnPI) * r) / 1000;
        Y = (628 + sin(i * turnPI) * r) / 1000;
        targetPos.at(0) = X;
        targetPos.at(1) = Y;
        targetPos.at(2) = Z;
        kinematicSolver->cartToJnt(initialJoint, targetPos, targetJoint);
        temp.motionType = ArmRobotMotionType::ServoJ;
        temp.acc = 30.00;
        temp.vel = 100.00;
        temp.blendRaidus = 0.00;
        temp.jointPosition = targetJoint;
        movePath.push_back(temp);
        robotDriver->moveArm(movePath);
        movePath.clear();
    }

    return 0;
}



