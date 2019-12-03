cmake_minimum_required(VERSION 3.7)
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Tiger.Common Lion.LocalStorageRepository.API Wolf.RobotDriver.API)
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES libcurl ffmpeg boost Qt5::Core Qt5::Gui Qt5::Test Qt5::X11Extras Qt5::Qml Qt5::Widgets)
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/tool.template.cmake)
