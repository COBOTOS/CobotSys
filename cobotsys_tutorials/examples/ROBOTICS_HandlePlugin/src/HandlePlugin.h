//
// Created by SCM on 19-11-19.
// Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSTUDIO_HandlePlugin_H
#define COBOTSTUDIO_HandlePlugin_H


#include <RobWorkStudioPlugin.hpp>
#include <rw/rw.hpp>
#include <rw/models/WorkCell.hpp>
#include <logger/Logger.h>
#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>
#include <LocalStorageRepositoryFactoryInterface.h>
#include <ForceSensorFactoryInterface.h>

#include <ForceSensorInterface.h>
#include <LocalStorageRepositoryAPIKey.h>
#include <boost/shared_ptr.hpp>
#include <thread>
#include <mutex>
#include <logger/GlogConfig.h>


using namespace TIGER_COMMON_NS_API;
using namespace LION_LSR_NS_API;
using namespace BAT_FORCESENSOR_NS_API;

namespace Ui {
    class HandlePluginUI;
}; // namespace Ui

class AR5Kinematics;

class HandlePlugin : public rws::RobWorkStudioPlugin {
Q_OBJECT
public:
    //! @brief Constructor.
    HandlePlugin();

    //! @brief Destructor.
    virtual ~HandlePlugin();

    //! @copydoc RobWorkStudioPlugin::initialize
    void initialize();

    void initWidget(QDockWidget *parent);

    //! @copydoc RobWorkStudioPlugin::open
    void open(rw::models::WorkCell *workcell) override;

    void close();

    void test();

    void dragHandle();

    void threadfunc();
    void start();
    void stop();
    void starttraj();

Q_SIGNALS:
    void updateCurrentQ();
    void updateView();

private Q_SLOTS:
    void doUpdateView();
    void doUpdateCurrentQ();

private:
    Ui::HandlePluginUI *ui;//视图
    AR5Kinematics* _kinematics;
    QTimer *_timer;
    QTimer *_timer2;
    boost::shared_ptr<ForceSensorInterface> forceSensor;
    std::vector<double> contactforce;
    rw::models::WorkCell::Ptr _workcell;
    rw::models::Device::Ptr _device;
    bool _exit;
    rw::math::Q _currentQ;
    rw::math::Q _cmdQ;
    rw::math::Q lastQ;
    std::mutex _mutex;
    rw::trajectory::QPath path;
    std::thread _tr;
    bool _once;
    rw::math::Wrench6D<> _initialWrench;
};

#endif //COBOTSTUDIO_SimplePlugin_H
