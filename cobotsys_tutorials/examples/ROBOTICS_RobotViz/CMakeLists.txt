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
        )

set(COBOTSYS_DEPENDED_3RD_MODULES Qt5::Core Qt5::Widgets)

set(QT_NO_KEYWORDS_ENABLE ON)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/qt.executable.cmake)