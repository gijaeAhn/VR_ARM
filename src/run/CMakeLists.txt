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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(${ROOT_DIRECOTORY}
                    ${rclcpp_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    ${ZeroMQ_INCLUDE_DIRS}
                    utilities/include
                    vr_interface)

add_executable(rmd_run src/rmd_run.cpp)
add_executable(motor_run src/motor_run.cpp)
add_executable(dyna_run src/dynamixel_run.cpp)
add_executable(vr_interface_run src/vr_interface_run.cpp)

target_compile_features(rmd_run PRIVATE cxx_std_17)
target_compile_features(motor_run PRIVATE cxx_std_17)
target_compile_features(dyna_run PRIVATE cxx_std_17)
target_compile_features(vr_interface_run PRIVATE cxx_std_17)


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

target_link_libraries(vr_interface_run
                      utilities
                      libzmq-static
                      vr_interface)

apply_global_cxx_flags_to_all_targets()
remove_flag_from_file(dyna_run src/dynamixel_run.cpp -Werror)

