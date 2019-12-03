cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Peacock.RobWork.Core Tiger.Common)
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.template.cmake)