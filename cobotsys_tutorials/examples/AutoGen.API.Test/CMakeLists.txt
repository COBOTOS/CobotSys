cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Tiger.Common Tiger.DBus-CXX)
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES boost)

include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/dbus_systemapi.template.cmake)







