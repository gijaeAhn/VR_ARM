cmake_minimum_required(VERSION 3.23.0)
project(robot)

option(PYTHON_BINDINGS "Building Python bindings" OFF)
option(BUILD_TESTING "Build unit and integration tests" OFF)
option(SETUP_TEST_IFNAME "Set-up the test VCAN interface automatically" OFF)

if(NOT UNIX)
    message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${ROOT_DIRECTORY})

find_package(Eigen3 REQUIRED)

add_library(robot src/dummy.cpp)

target_link_libraries(robot Eigen3::Eigen)
