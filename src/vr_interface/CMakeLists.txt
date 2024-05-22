cmake_minimum_required(VERSION 3.23.0)
project(run)

option(PYTHON_BINDINGS "Building Python bindings" OFF)
option(BUILD_TESTING "Build unit and integration tests" OFF)
option(SETUP_TEST_IFNAME "Set-up the test VCAN interface automatically" OFF)

if(NOT UNIX)
    message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ZeroMQ REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



add_library(vr_interface ControllerListener.cpp)
ament_target_dependencies(vr_interface rclcpp
                                       std_msgs
                                       geometry_msgs
                                       tf2
                                       tf2_ros
                                        ZeroMQ )

target_include_directories(vr_interface PUBLIC ${ROOT_DIRECTORY})

