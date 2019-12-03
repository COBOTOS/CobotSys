cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Lion.LocalStorageRepository.API )
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES opencv pcl Qt5::Core Qt5::Widgets Qt5::Qml cppunit oracle)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)