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
add_executable(motor_run src/motor_run.cpp)
add_executable(dyna_run src/dynamixel_run.cpp
        ../math/transform.hpp)

target_include_directories(rmd_run PRIVATE    ${ROOT_DIRECTORY})
target_include_directories(motor_run PRIVATE    ${ROOT_DIRECTORY})
target_include_directories(dyna_run PRIVATE ${ROOT_DIRECTORY})

target_link_libraries(rmd_run
                      rmd_driver
                      utilities
                      libzmq-static)

target_link_libraries(motor_run
                      utilities
                      libzmq-static)
                      
target_link_libraries(dyna_run
                      utilities
                      dynamixel_sdk
                      libzmq-static)   

target_compile_features(rmd_run PRIVATE cxx_std_17)
target_compile_features(motor_run PRIVATE cxx_std_17)
target_compile_features(dyna_run PRIVATE cxx_std_17)

apply_global_cxx_flags_to_all_targets()
remove_flag_from_file(dyna_run src/dynamixel_run.cpp -Werror)

