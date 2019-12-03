cmake_minimum_required(VERSION 3.7)

set(COBOTSYS_DEPENDED_MODULES
        Ox.RobotWork.RW
        Ox.RobotWork.RWlibs
        Peacock.RobotViz.FrameWork
        Peacock.RobotVizPlugin.Jog
        Peacock.RobotVizPlugin.Log
        Peacock.RobotVizPlugin.Planning
        Peacock.RobotVizPlugin.PlayBack
        Peacock.RobotVizPlugin.TreeView
        Tiger.Common
        Tiger.DBus-CXX
        Bat.ForceSensor.API
        Bat.ForceSensor.PluginAPI
        Bat.ForceSensor.Common
        Lion.LocalStorageRepository.API
        Wolf.ARKinematics
        )

set(COBOTSYS_DEPENDED_3RD_MODULES Qt5::Core Qt5::Widgets boost)

set(QT_NO_KEYWORDS_ENABLE ON)

include_directories(include)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/qt.template.cmake)
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)