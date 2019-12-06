cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Tiger.Common Eagle.Camera3D.API Eagle.Camera3D.PluginAPI Lion.LocalStorageRepository.API
        Ox.RobotWork.RW
        Ox.RobotWork.RWlibs
        Peacock.RobotViz.FrameWork
        Peacock.RobotVizPlugin.Jog
        Peacock.RobotVizPlugin.Log
        Peacock.RobotVizPlugin.Planning
        Peacock.RobotVizPlugin.PlayBack
        Peacock.RobotVizPlugin.TreeView)
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES opencv basler opencv realsense2 Qt5::Core Qt5::Widgets)

set(QT_NO_KEYWORDS_ENABLE ON)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/qt.template.cmake)
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)
