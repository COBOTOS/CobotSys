//
// Created by cobot on 19-11-11.
//

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

#include <logger/Logger.h>

using namespace rw;
using namespace rw::loaders;

int main(int argc, char** argv){

    auto workcell = WorkCellLoader::Factory::load("/home/cobot/cobotsys_tutorials/SimplePlugin/data/robot/AR_AR5/AR_AR5.robot.xml");
    auto ar5 = workcell->getDevices().at(0);
    auto state = workcell->getDefaultState();
    math::Q q(6,10,20,30,40,50,60);
    q *= math::Deg2Rad;
    ar5->setQ(q, state);

    auto trans= ar5->baseTend(state);
    LOG_INFO << "Transform3D of base to end is:" << trans;

    auto jcb = ar5->baseJend(state);

    LOG_INFO << "Jacobian of base to end is :" <<jcb;
}
