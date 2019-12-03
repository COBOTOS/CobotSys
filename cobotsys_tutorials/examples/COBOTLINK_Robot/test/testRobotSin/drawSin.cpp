/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-12-2         shengchengmin
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
//    boost::shared_ptr<KinematicSolverFactoryInterface> kinematicSolverFactory = KinematicSolverFactoryInterface::create();
//    boost::shared_ptr<KinematicSolverInterface> kinematicSolver = kinematicSolverFactory->createKinematicSolver(robotName);

    //连接机器人
    RobotStatus status = robotDriver->connect(ip);
    LOG_INFO << "robot connect Status:" << RobotStatusEnum2String(status);

    //让机器人走两次正弦波
    std::chrono::duration<double> dur(0.008);
    std::vector<double> initQ(robotDriver->getRobotJoints());
    std::vector<double> cmdQ = initQ;
    std::vector<ArmRobotWayPoint> movePath;
    ArmRobotWayPoint temp;
    for(int ii = 0; ii < 2;ii++){
        for(double t = 0.0;t < 5;t += 0.008){
            auto time_now = std::chrono::high_resolution_clock::now();
            for(int mm = 0; mm < 6; mm++){
                cmdQ.at(mm) = initQ.at(mm) + 0.5 * sin(0.1 * 2 * M_PI * t);
            }
            temp.motionType = ArmRobotMotionType::MoveJ;
            temp.acc = 10.00;
            temp.vel = 20.00;
            temp.blendRaidus = 0.00;
            temp.jointPosition = cmdQ;
            movePath.push_back(temp);
            robotDriver->moveArm(movePath);
            movePath.clear();
            std::this_thread::sleep_until(time_now + dur);
        }
    }


    return 0;
}



