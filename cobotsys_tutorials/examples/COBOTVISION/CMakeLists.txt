cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Eagle.Vision.PluginAPI Eagle.Vision.API Eagle.Camera3D.API Lion.LocalStorageRepository.API
         Eagle.EagleData.Common Eagle.VisionGrasp.Common )
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES opencv pcl Qt5::Core Qt5::Widgets Qt5::Qml cppunit vtk)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)