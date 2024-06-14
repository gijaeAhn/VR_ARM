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
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


find_package(Eigen3 REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(Threads REQUIRED)

include_directories(${ZeroMQ_INCLUDE_DIRS})


add_library(robot src/robot_body.cpp
                  src/robot_builder.cpp)

target_link_libraries(robot Eigen3::Eigen
                            math
                            libzmq-static
                            pthreads)

target_include_directories(robot PUBLIC ${math_INCLUDE_DIR}
                                        ${utilities_INCLUDE_DIR}
                                        )
