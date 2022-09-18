cmake_minimum_required(VERSION 3.16)
project(rmpcpp_planner LANGUAGES CXX CUDA)


find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

SET(CMAKE_CUDA_ARCHITECTURES 61)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS " -Wall -Wextra")
set(CMAKE_BUILD_TYPE Release)

#####################################
#### COPIED FROM NVBLOX:
# Suppress spammy Eigen CUDA warnings.
# "expt-relaxed-constexpr" allows sharing constexpr between host and device
# code.
# "display_error_number" shows a warning number with all warnings, and the
# rest is just suppressing specific warnings from Eigen. Note that the numbers
# keep changing with every CUDA release so this list is a bit arbitrary.
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr  --generate-line-info -lineinfo -Xcudafe --display_error_number -Xcudafe --diag_suppress=2977  -Xcudafe --diag_suppress=3057  -Xcudafe --diag_suppress=3059 ")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC -arch=sm_61")
MESSAGE(WARN ${CUDA_NVCC_FLAGS})
# New warning numbers above CUDA 11.2.
if (CUDA_VERSION_MAJOR EQUAL 11 AND CUDA_VERSION_MINOR GREATER_EQUAL 2)
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=20012 -Xcudafe --diag_suppress=20011 -Xcudafe --diag_suppress=20014")
endif ()
######################################
find_package(CUDA REQUIRED)
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})


set(SOURCES
        src/core/planner_rmp.cc
        src/core/world.cc
        src/testing/worldgen.cc
        src/core/trajectory_rmp.cc
        src/policies/simple_ESDF.cc
        src/policies/raycasting_CUDA.cu
        src/policies/lidarray_CUDA.cu
        src/testing/parser.cc
        src/core/world_rmp.cc)

set(TESTSOURCES
        src/testing/tester.cc
        src/testing/data_export.cc)


# HEADERS
include_directories(include)

# NVBLOX
add_subdirectory(nvblox/nvblox)

# BOOST
find_package(Boost COMPONENTS program_options REQUIRED)

# RMPCPP
include_directories(../rmpcpp/include)


# Main executable
cs_add_executable(${PROJECT_NAME}_runtest
        ${SOURCES}
        ${TESTSOURCES}
        tests/run_test.cc)
target_link_libraries(${PROJECT_NAME}_runtest
        nvblox_lib
        Boost::program_options
        Eigen3::Eigen)


cs_add_executable(${PROJECT_NAME}_lidarnode
        ${SOURCES}
        ${TESTSOURCES}
        ${COMPARISON_SOURCES}
        src/node/lidar_cloud_node.cc)
target_link_libraries(${PROJECT_NAME}_lidarnode
        nvblox_lib
        Boost::program_options
        Eigen3::Eigen)


# Copies the binaries from the catkin devel folder to the ./build folder
add_custom_target(build-time-make-directory ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rmpcpp/rmpcpp_planner/build/)
add_custom_command(TARGET ${PROJECT_NAME}_runtest
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}_runtest>
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/rmpcpp/rmpcpp_planner/build/)

cs_install()
cs_export()
