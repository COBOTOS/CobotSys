//
// Created by cobot on 19-11-11.
//

#include <rw/math.hpp>
#include <logger/Logger.h>
/*
 * 将RPY转为3*3的正交旋转矩阵
 * */
using namespace rw;

int main(int argc, char** argv){

    //比如roll,pith,yaw分别为
    math::RPY<> rpy(10.0/180.0*M_PI,20.0/180.0*M_PI,30.0/180.0*M_PI);
    math::Quaternion<> quat(rpy.toRotation3D());

    LOG_INFO << "quaternion is:"<<quat;

}