cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Tiger.Common Eagle.Camera3D.API Eagle.Camera3D.PluginAPI Lion.LocalStorageRepository.API)
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES opencv basler opencv realsense2)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)
