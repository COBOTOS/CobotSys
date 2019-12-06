//
// Created by 杨帆 on 17-7-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSTUDIO_SimplePlugin_H
#define COBOTSTUDIO_SimplePlugin_H


#include <RobWorkStudioPlugin.hpp>
#include <rw/models/WorkCell.hpp>
#include <logger/Logger.h>
namespace Ui {
    class SimplePluginUI;
}; // namespace Ui

class SimplePlugin : public rws::RobWorkStudioPlugin {
Q_OBJECT
public:
    //! @brief Constructor.
    SimplePlugin();

    //! @brief Destructor.
    virtual ~SimplePlugin();

    //! @copydoc RobWorkStudioPlugin::initialize
    void initialize();

    void initWidget(QDockWidget *parent);

    void close();

    void test();
private:
    Ui::SimplePluginUI *ui;//视图

    QTimer *_timer;

    rw::models::WorkCell::Ptr _workcell;
    rw::models::Device::Ptr _device;
};

#endif //COBOTSTUDIO_SimplePlugin_H
