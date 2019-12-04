cmake_minimum_required(VERSION 3.7)

set(COBOTSYS_DEPENDED_MODULES Tiger.Common Eagle.Camera3D.API Eagle.Camera3D.PluginAPI Lion.LocalStorageRepository.API)

set(COBOTSYS_DEPENDED_3RD_MODULES visp eigen3 opencv pcl vtk)

set(QT_NO_KEYWORDS_ENABLE ON)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)
