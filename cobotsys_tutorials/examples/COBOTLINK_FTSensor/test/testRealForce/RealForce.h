/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-8-16           tangmingwu
============================================================== **/


#ifndef PROJECT_REALFORCE_H
#define PROJECT_REALFORCE_H

//接口和宏

#include <RobotDriverInterface.h>
#include <RobotDriverFactoryInterface.h>
#include <ForceSensorInterface.h>
#include <ForceSensorFactoryInterface.h>
#include <ToolSettingsStruct.h>
#include <KinematicSolverInterface.h>
#include <KinematicSolverFactoryInterface.h>


//std 和 boost
#include <string>
#include <vector>
#include <map>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

//其他第三方库
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>

#include <rw/math.hpp>
#include <rw/trajectory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/invkin.hpp>
#include <rw/invkin/ClosedFormIK.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

using namespace rw::invkin;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::trajectory;
using namespace rw::common;
using namespace rw::proximity;

using namespace WOLF_KINEMATICSOLVER_NS_API;
using namespace WOLF_ROBOTDRIVER_NS_API;
using namespace BAT_FORCESENSOR_NS_API;

class RealForce{
public:
    RealForce();

    ~RealForce();

    void setRobotName(const std::string& strRobotName);

    void setForceSensorName(const std::string& strForceSensorName);

    void setToolSettings(boost::shared_ptr<CHEETAH_DATA_NS_API::ToolSettings> toolSettings);

    std::vector<double> getRealInterfaceForce(const std::vector<double>& forceFromSensor,
                                              const std::vector<double >& currentPose);
    void execute();
private:
    boost::shared_ptr<CHEETAH_DATA_NS_API::ToolSettings> _polishToolSettings;
    Wrench6D<double> _gravity;
    Transform3D<double> _CenterOfGravity;

    boost::shared_ptr<KinematicSolverInterface> _ikSolver;
    boost::shared_ptr<RobotDriverInterface> _robotDriver;
    boost::shared_ptr<ForceSensorInterface> _forceSensor;

    std::vector<double> _absError;
};


#endif //PROJECT_REALFORCE_H
