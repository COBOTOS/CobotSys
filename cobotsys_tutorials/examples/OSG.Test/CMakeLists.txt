cmake_minimum_required(VERSION 3.7)
##导入sdk配置就可以开始使用开源库了
set(COBOTSYS_DEPENDED_MODULES  Lion.LocalStorageRepository.API Tiger.Common)
set(COBOTSYS_DEPENDED_3RD_MODULES  Qt5::Core Qt5::Widgets Qt5::Gui Qt5::Network Qt5::Widgets Qt5::Sql Qt5::OpenGL osg boost )

#set(COBOTSYS_LISTENER_GENERATE_DIRS test/testListener)
#include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/listener_template.cmake)
#include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/component.template.cmake)
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)
