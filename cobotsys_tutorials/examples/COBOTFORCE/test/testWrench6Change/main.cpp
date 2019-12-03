/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-8-20           tangmingwu
============================================================== **/


#include <rw/math.hpp>
#include <logger/Logger.h>
using namespace rw::math;

std::vector<double> getTransForce(Wrench6D<double> gravity,
                                  Transform3D<double> transGracityCenterinTCP)
{
    std::vector<double> force;

    Wrench6D<double> gravityOffset = transGracityCenterinTCP * gravity;

    for(int i = 0;i<6;++i)
    {
        force.push_back(gravityOffset[i]);
    }

    LOG_INFO << force;
    return force;
}

int main()
{
    Wrench6D<double> gravity;
    Transform3D<double> transGracityCenterinTCP;

    gravity = Wrench6D<>(0,0,-1,-0,0,-0);
    transGracityCenterinTCP = Transform3D<double>(Vector3D<double>(0,1,0));
    getTransForce(gravity,transGracityCenterinTCP);

    gravity = Wrench6D<>(0,-1,-1,0,0,-0);
    transGracityCenterinTCP = Transform3D<double>(Vector3D<double>(0,1,0));
    getTransForce(gravity,transGracityCenterinTCP);

    gravity = Wrench6D<>(-1,-1,-1,0,0,-0);
    transGracityCenterinTCP = Transform3D<double>(Vector3D<double>(0,1,0));
    getTransForce(gravity,transGracityCenterinTCP);

    return 0;
}