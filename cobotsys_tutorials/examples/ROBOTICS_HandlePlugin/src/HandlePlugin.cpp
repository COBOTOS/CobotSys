//
// Created by SCM on 2019-11-19.
// Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "HandlePlugin.h"
#include <RobWorkStudio.hpp>

#include <logger/Logger.h>

#include "ui_HandlePlugin.h"
#include <QTimer>
#include <mutex>
#include <AR5Kinematics.h>

using namespace rw;


HandlePlugin::HandlePlugin() :
        rws::RobWorkStudioPlugin(tr("HandlePlugin"),QIcon("SP")) {
    _exit = false;
    _kinematics = new AR5Kinematics();
    initWidget(this);
    boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
    forceSensor = forceSensorFactory->createForceSensor("OptoForceSensor");
    SensorInfo sensorInfo;
    sensorInfo.uri = "192.168.1.1";
    ForceSensorStatus forceSensorStatus;
    forceSensorStatus = forceSensor->connect(sensorInfo);
    LOG_INFO << "connect forceSensor result=" << ForceSensorStatusEnum2String(forceSensorStatus);
//    _tr = std::thread(&HandlePlugin::threadfunc,this);
//    _tr.detach();
    LOG_INFO << "construct HandlePlugin";
    _once=true;
}

HandlePlugin::~HandlePlugin() {
}

void HandlePlugin::initWidget(QDockWidget *parent) {
    _workcell = nullptr;
    _device = nullptr;
    ui = new Ui::HandlePluginUI();
    ui->setupUi(parent);
    connect(ui->btn_test, &QPushButton::released,
            this, &HandlePlugin::test);
    connect(ui->btn_start, &QPushButton::released,this, &HandlePlugin::start);
    connect(ui->btn_stop, &QPushButton::released,this,&HandlePlugin::stop);
    connect(this, SIGNAL(updateCurrentQ()),this,SLOT(doUpdateCurrentQ()),Qt::DirectConnection);
    connect(this, SIGNAL(updateView()),this,SLOT(doUpdateView()),Qt::DirectConnection);
    _timer = new QTimer();
    _timer2 = new QTimer();
    _timer2->setInterval(0.2);
//    connect(_timer,&QTimer::timeout,this,&HandlePlugin::starttraj);
    connect(_timer2,&QTimer::timeout,this,&HandlePlugin::dragHandle);
}

void HandlePlugin::initialize() {
    
}

void HandlePlugin::doUpdateView() {
    if(_cmdQ.size()!=6){
        LOG_ERROR << "The size of commandq is not 6";
        return;
    }
    auto state = getRobWorkStudio()->getState();
    _mutex.lock();
    _device->setQ(_cmdQ,state);
    _mutex.unlock();
    getRobWorkStudio()->setState(state);
}

void HandlePlugin::doUpdateCurrentQ() {
    auto state = getRobWorkStudio()->getState();
    _mutex.lock();
    _currentQ = _device->getQ(state);
    _mutex.unlock();
}

void HandlePlugin::open(rw::models::WorkCell *workcell) {
    LOG_INFO << "A workcell is loaded.";
    _workcell = workcell;
    auto devices = _workcell->getDevices();
    if(!devices.empty()){
        _device = devices.at(0);
        _timer2->start();
    }
}

void HandlePlugin::close(){

}

void HandlePlugin::start(){
    _timer->start(100);
}

void HandlePlugin::starttraj(){
    if(_currentQ.size()!=6 || (_currentQ - lastQ).norm2() > 1e-6){
        return;
    }else{
        path.push_back(_currentQ);
        lastQ = _currentQ;
    }
}

void HandlePlugin::stop(){
//    _timer->stop();
}

void HandlePlugin::test(){
    if(!_device){
        LOG_ERROR <<"No device";
        return;
    }
}

void HandlePlugin::dragHandle(){
    rw::math::Q currentQ;
    rw::math::Q cmdQ;
    Q_EMIT updateCurrentQ();
    currentQ = _currentQ;
    contactforce = forceSensor->getSensorData();
    auto curFT = rw::math::Wrench6D<>(
            contactforce.at(0),contactforce.at(1),contactforce.at(2),
            contactforce.at(3),contactforce.at(4),contactforce.at(5));
    if(_once){
        _initialWrench = curFT;
        curFT = rw::math::Wrench6D<>();
        _once = false;
    }else{
        curFT = curFT - _initialWrench;
    }
    if(curFT.force().normInf()< 1.0){
        curFT = rw::math::Wrench6D<>();
    }
    rw::math::Vector3D<> offsetP(curFT.force()(0),curFT.force()(1),curFT.force()(2));
    offsetP=offsetP*0.05*0.01;

    auto currentTranform3D = _kinematics->baseTend(currentQ);
    auto commandTranform3D = currentTranform3D * rw::math::Transform3D<>(offsetP);
    cmdQ = _kinematics->getQ(commandTranform3D,currentQ);
    _cmdQ = cmdQ;
    Q_EMIT updateView();
}