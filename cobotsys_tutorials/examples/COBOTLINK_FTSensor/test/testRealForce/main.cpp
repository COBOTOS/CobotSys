/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-8-19           tangmingwu
============================================================== **/


#include "RealForce.h"
#include <ToolSettingsStruct.h>

int main()
{
    boost::shared_ptr<CHEETAH_DATA_NS_API::ToolSettings> toolSettings = boost::make_shared<CHEETAH_DATA_NS_API::ToolSettings>();
    toolSettings->gravityCenterX = 0.000311452;
    toolSettings->gravityCenterY = -0.001584223;
    toolSettings->gravityCenterZ = 0.064038554;
    toolSettings->weight = 11.61531667;

    boost::shared_ptr<RealForce> realForce = boost::make_shared<RealForce>();

    realForce->setRobotName("ur5");
    realForce->setForceSensorName("DynPickFTSensor");
    realForce->setToolSettings(toolSettings);
    realForce->execute();

    return 0;
}

