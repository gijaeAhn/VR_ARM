cmake_minimum_required(VERSION 3.16.3)
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


# find_package(Threads REQUIRED)
# find_package(Eigen3)
# set(EIGEN_PACKAGE EIGEN3)
# if(NOT EIGEN3_FOUND)
#   find_package(cmake_modules REQUIRED)
#   find_package(Eigen REQUIRED)
#   set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
#   set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
#   set(EIGEN_PACKAGE Eigen)
# endif()

# add_library(robot SHARED ${SRC_FILES}
#         include/robot_IK.h
#         include/robot_COMMON.h
# )



# target_compile_options(robot PRIVATE -Wall -Werror)


add_executable(rmd_run src/rmd_run.cpp)


target_include_directories(rmd_run PRIVATE    ${ROOT_DIRECTORY})

target_link_libraries(rmd_run
                      rmd_driver
                      utilities
                      libzmq-static)




target_compile_features(rmd_run PRIVATE cxx_std_17)
