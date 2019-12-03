//
// Created by cobot on 19-11-11.
//

#include <rw/math.hpp>
#include <logger/Logger.h>

using namespace rw;

int main(int argc, char** argv){
    math::Vector3D<> xyz(0.1,0.2,0.5);
    math::RPY<> rpy(10.0/180.0*M_PI,20.0/180.0*M_PI,30.0/180.0*M_PI);
    math::Transform3D<> transform3D(xyz,rpy.toRotation3D());
    LOG_INFO << "Transform3D is:"<<transform3D;
}