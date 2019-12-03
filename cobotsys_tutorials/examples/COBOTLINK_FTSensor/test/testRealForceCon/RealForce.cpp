/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-8-16           tangmingwu
============================================================== **/

#include "RealForce.h"
#include <math.h>

using namespace rw::math;
const double constPI = 3.1415926;
RealForce::RealForce() {
    _absError = {0.343625611,1.836905,12.62713833,0.000342267,0.192617222,0.001056442};
    LOG_INFO <<"errorForce: " << _absError;
}

RealForce::~RealForce() {
    _robotDriver->disconnect();
    _forceSensor->disconnect();
}

void RealForce::setRobotName(const std::string &strRobotName) {
    boost::shared_ptr<KinematicSolverFactoryInterface> ikFactory = KinematicSolverFactoryInterface::create();
    _ikSolver = ikFactory->createKinematicSolver("ur5");

    boost::shared_ptr<RobotDriverFactoryInterface> rdFactory = RobotDriverFactoryInterface::create();
    _robotDriver = rdFactory->createRobotDriver("ur5");

    _robotDriver->connect("192.168.100.5");

}

void RealForce::setForceSensorName(const std::string& strForceSensorName) {
    boost::shared_ptr <ForceSensorFactoryInterface> factory = ForceSensorFactoryInterface::create();
    _forceSensor = factory->createForceSensor(strForceSensorName);

    SensorInfo sensorInfo;
    sensorInfo.uri = "/dev/ttyUSB0";
    _forceSensor->connect(sensorInfo);
}

void RealForce::setToolSettings(boost::shared_ptr<CHEETAH_DATA_NS_API::ToolSettings> toolSettings){
    _polishToolSettings = toolSettings;
    _gravity = Wrench6D<>(0,0,-_polishToolSettings->weight,-0,0,-0);
    _CenterOfGravity = Transform3D<double>(Vector3D<double>(_polishToolSettings->gravityCenterX,
                                                            _polishToolSettings->gravityCenterY,
                                                            _polishToolSettings->gravityCenterZ));
}

std::vector<double> RealForce::getRealInterfaceForce(
        const std::vector<double>& forceFromSensor,const std::vector<double >& currentPose)
{
    std::vector<double> force;
    Wrench6D<double> force6D;
    auto currentPose3D = Transform3D<>(Vector3D<double>(currentPose[0],currentPose[1],currentPose[2]),
                                       RPY<double>(currentPose[5],currentPose[4],currentPose[3]));//rpy转为标准旋转变换矩阵R，RPY::toRotation3D
    for(std::size_t i = 0;i<6;++i)
    {
        force6D[i] = forceFromSensor[i];
    }
    double theta = -135/180.0*constPI;
    Rotation3D<double> rotationForceSensor(cos(theta),-sin(theta),0,
                                           sin(theta),cos(theta), 0,
                                           0,         0,          1);

    currentPose3D.R() = currentPose3D.R() * rotationForceSensor;
    /// -----

    Transform3D<double> transGracityCenterinTCP = Transform3D<double>(
            _CenterOfGravity.P(), currentPose3D.R().inverse());//

    Wrench6D<double> gravityOffset = transGracityCenterinTCP * _gravity; //gravityOffset是相对于力传感器坐标的

    auto realForce = force6D-gravityOffset;
    for(std::size_t i = 0;i<6;++i)
    {
        force.push_back(realForce[i]);
    }
    return force;
}

void RealForce::execute() {
    std::vector<double>  dJoints = _robotDriver->getRobotJoints();

//    LOG_INFO << "dJoints: " << dJoints;
    std::vector<double> currentPose;
    _ikSolver->jntToCart(dJoints,currentPose);
//    LOG_INFO << "currentPose: " << currentPose;

    std::vector<double> forceFromSensor{0,0,0,0,0,0};
    std::vector<double> tempForce;
    int i = 0;
    for(;i<10; ++i)
    {
        usleep(20000);
        tempForce = _forceSensor->getSensorData();
        for(int j=0; j<6; j++) {
//            tempForce[j] -= _absError[j];
            forceFromSensor[j] += tempForce[j];
        }
    }
    for(int j=0; j<6; j++) {
        forceFromSensor[j] /= i;
    }

    LOG_INFO << "forceFromSensor: " << forceFromSensor;

    for(int j=0; j<6; j++) {
        forceFromSensor[j] -= _absError[j];
    }

    LOG_INFO << "forceFromSensor - absError: " << forceFromSensor;

    std::vector<double> currentForce = getRealInterfaceForce(forceFromSensor,currentPose);
    LOG_INFO << "currentForce: " << currentForce;
}