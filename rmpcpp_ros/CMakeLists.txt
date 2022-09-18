cmake_minimum_required(VERSION 2.8.3)
project(rmpcpp_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)


#############
# LIBRARIES #
#############
cs_add_executable(rmpcpp_ros_example src/example.cc
        src/arrow_renderer.cc
        src/color_map.cc)


##########
# EXPORT #
##########
cs_install()
cs_export()
