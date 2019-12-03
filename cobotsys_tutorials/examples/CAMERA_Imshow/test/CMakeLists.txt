cmake_minimum_required(VERSION 3.7)
##导入sdk配置就可以开始使用开源库了
set(COBOTSYS_DEPENDED_MODULES  Tiger.Utility Ox.RobotWork.RW)
set(COBOTSYS_DEPENDED_3RD_MODULES  aubo Qt5::Core Qt5::Gui Qt5::Qml Qt5::Widgets )

#set(COBOTSYS_LISTENER_GENERATE_DIRS test/testListener)
#include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/listener_template.cmake)
#include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/component.template.cmake)
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)
