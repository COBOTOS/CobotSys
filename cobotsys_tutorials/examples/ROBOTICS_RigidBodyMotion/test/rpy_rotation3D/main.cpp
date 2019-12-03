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

    math::Rotation3D<> rotation3D=rpy.toRotation3D();

    LOG_INFO << "Rotation3D is:"<<rpy.toRotation3D();
    math::Vector3D<> xAxis = rotation3D.getCol(0);
    LOG_INFO << "x axis is:"<<xAxis << " | The length of x axis is:"<< xAxis.norm2();

    math::Vector3D<> yAxis = rpy.toRotation3D().getCol(1);
    LOG_INFO << "y axis is:"<<yAxis << " | The length of y axis is:"<< yAxis.norm2();

    LOG_INFO << "z axis is:"<<rotation3D.getCol(2)<< " | The length of z axis is:"<< rotation3D.getCol(2).norm2();

    LOG_INFO << "x轴和y轴垂直吗？ math::dot(xAxis,yAxis) is zero? "<<math::dot(xAxis,yAxis);

}