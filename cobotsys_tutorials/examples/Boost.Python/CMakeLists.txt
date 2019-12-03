cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES Tiger.Common)
set(COBOTSYS_DEPENDED_3RD_MODULES boost)
#导入模板
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/pythonso.template.cmake)