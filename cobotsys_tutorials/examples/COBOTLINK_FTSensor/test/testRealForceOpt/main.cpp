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
    toolSettings->gravityCenterX = 0.001283782;
    toolSettings->gravityCenterY = -0.009547505;
    toolSettings->gravityCenterZ = 0.034642728;
    toolSettings->weight = 12.88179444;

    boost::shared_ptr<RealForce> realForce = boost::make_shared<RealForce>();

    realForce->setRobotName("ur3");
    realForce->setForceSensorName("OptoForceSensor");
    realForce->setToolSettings(toolSettings);
    realForce->execute();

    return 0;
}

