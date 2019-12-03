/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-11-10           杨帆
============================================================== **/

#include <QApplication>
#include "RobWorkStudio.hpp"
#include <rw/RobWork.hpp>
#include <rw/common/ProgramOptions.hpp>
#include <RobWorkStudioConfig.hpp>
#include <gui/GuiApplication.h>
#include <base/Application.h>

#include <ShowLog.hpp>
#include <PlayBack.hpp>
#include <Planning.hpp>
#include <TreeView.hpp>
#include <Jog.hpp>
#include "../../src/SimplePlugin.h"

using namespace rw::common;
using namespace cobotsys::core;


int main(int argc, char** argv)
{

    ///init cobotsys
    GuiApplication app(argc, argv);
    rw::RobWork::init(argc,argv);
    ProgramOptions poptions("RobotViz", RW_VERSION);
    PropertyMap map = poptions.getPropertyMap();
    GetApplication().NewDocument( "SDI" );
    rws::RobWorkStudio *rwstudio = new rws::RobWorkStudio(map);
    rwstudio->InitView();
    app.SetWidget( rwstudio );
    rwstudio->addPlugin(new rws::Jog(), true, Qt::LeftDockWidgetArea, true);
    rwstudio->addPlugin(new rws::TreeView(), true, Qt::LeftDockWidgetArea, true);
    rwstudio->addPlugin(new rws::PlayBack(), true, Qt::BottomDockWidgetArea, true);
    rwstudio->addPlugin(new rws::ShowLog(), true, Qt::BottomDockWidgetArea, true);
    rwstudio->addPlugin(new rws::Planning(), true, Qt::RightDockWidgetArea, true);
    rwstudio->addPlugin(new SimplePlugin(), true, Qt::RightDockWidgetArea, true);
    return app.Execute();

}