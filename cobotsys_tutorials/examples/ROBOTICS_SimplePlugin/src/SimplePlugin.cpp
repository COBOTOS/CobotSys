//
// Created by 杨帆 on 17-7-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "SimplePlugin.h"
#include <RobWorkStudio.hpp>

#include <logger/Logger.h>

#include "ui_SimplePlugin.h"
#include <QTimer>

using namespace rw;

SimplePlugin::SimplePlugin() :
        rws::RobWorkStudioPlugin(tr("SimplePlugin"),QIcon("SP")) {
    initWidget(this);
    LOG_INFO << "construct SimplePlugin";
}

SimplePlugin::~SimplePlugin() {
}

void SimplePlugin::initWidget(QDockWidget *parent) {
    _workcell = nullptr;
    _device = nullptr;
    ui = new Ui::SimplePluginUI();
    ui->setupUi(parent);
    connect(ui->btn_test, &QPushButton::released,
            this, &SimplePlugin::test);

    _timer = new QTimer();
    connect(_timer,&QTimer::timeout,this,&SimplePlugin::test);

}

void SimplePlugin::initialize() {
    
}

void SimplePlugin::open(rw::models::WorkCell *workcell) {
    LOG_INFO << "A workcell is loaded.";
    _workcell = workcell;
    auto devices = _workcell->getDevices();
    if(!devices.empty()){
        _device = devices.at(0);
    }
}

void SimplePlugin::close(){

}

void SimplePlugin::test(){
    if(!_device){
        LOG_ERROR <<"No device";
        return;
    }
    auto state = getRobWorkStudio()->getState();
    rw::math::Q homeQ(6,10,20,30,40,50,60);
    homeQ *= math::Deg2Rad;
    _device->setQ(homeQ,state);
    getRobWorkStudio()->setState(state);

}