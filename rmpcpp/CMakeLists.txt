cmake_minimum_required(VERSION 2.8.3)
project(rmpcpp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_executable(rmpcpp_example example.cc)

##########
# EXPORT #
##########
cs_install()
cs_export()
