/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 8/19/19                 liuzhongxin
============================================================== **/


#include <ForceSensorInterface.h>
#include <ForceSensorFactoryInterface.h>
#include <vector>

using namespace BAT_FORCESENSOR_NS_API;
using namespace std;
int main()
{
    boost::shared_ptr<ForceSensorInterface> forceSensor;
    boost::shared_ptr <ForceSensorFactoryInterface> factory = ForceSensorFactoryInterface::create();
    forceSensor = factory->createForceSensor("DynPickFTSensor");

    SensorInfo sensorInfo;
    sensorInfo.uri = "/dev/ttyUSB0";
    forceSensor->connect(sensorInfo);

    vector<double> force;
    vector<double> temp;

    for(int i=0; i<6; i++)
    {
        force.push_back(0);
    }

    int i=0;
    for(; i<1000;)
    {
        usleep(20000);
        temp = forceSensor->getSensorData();

        for(int j=0; j<6; j++)
        {
            force[j]+=temp[j];
        }
        i++;
    }

    LOG_INFO <<"i:" << i;
    for(int j=0; j<6; j++)
    {
        force[j]/=i;
    }
    LOG_INFO <<"force:" << force;

}