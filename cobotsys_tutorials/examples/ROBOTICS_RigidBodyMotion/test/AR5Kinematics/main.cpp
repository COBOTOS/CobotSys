//
// Created by cobot on 19-11-11.
//

#include <rw/math.hpp>
#include <logger/Logger.h>
using namespace rw;

int main(int argc, char** argv){
    double a[6]={0, 0,0.528,0.447,0,0};
    double alpha[6]={M_PI, M_PI_2,0,M_PI,-M_PI_2,M_PI_2};
    double d[6]={0,-0.145,0,0,0.139,0.114};
    double theta[6]={0,M_PI_2,-M_PI_2,0,-M_PI_2,0};
    math::Q q(6,0.0);
    math::Transform3D<> trans_baseToEnd;
    for(int ii=0;ii<6;ii++)
        trans_baseToEnd=trans_baseToEnd*math::Transform3D<>::DH(alpha[ii],a[ii],d[ii],theta[ii]+q(ii));
    trans_baseToEnd=trans_baseToEnd*math::Transform3D<>(math::Vector3D<>(0,0,0.087));
    LOG_INFO << trans_baseToEnd;
}