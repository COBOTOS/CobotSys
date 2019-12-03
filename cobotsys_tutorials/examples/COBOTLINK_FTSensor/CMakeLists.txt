cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Tiger.Common
        Tiger.DBus-CXX
        Lion.LocalStorageRepository.API
        Cheetah.CheetahData.API
        Peacock.RoboticsFramework.API
        Bat.ForceSensor.API
        Wolf.RobotDriver.API
        Wolf.KinematicSolver.API)
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES boost eigen3)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)
