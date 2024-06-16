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

include_directories(
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${ZeroMQ_INCLUDE_DIRS}
        ${ROOT_DIRECTORY}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# List of executables and their sources
set(EXECUTABLES
        rmd_run src/rmd_run.cpp
        motor_run src/motor_run.cpp
        dyna_run src/dynamixel_run.cpp
        vr_interface_run src/vr_interface_run.cpp
)

# Create each executable, set common properties, and link common libraries
foreach(EXEC_NAME SRC_FILE IN LISTS EXECUTABLES)
    add_executable(${EXEC_NAME} ${SRC_FILE})
    target_compile_features(${EXEC_NAME} PRIVATE cxx_std_17)
    target_include_directories(${EXEC_NAME} PUBLIC ${ROOT_DIRECTORY} ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

# Specific target link libraries and directories
target_link_libraries(rmd_run PUBLIC rmd_driver utilities libzmq-static)
target_link_directories(rmd_run PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})

target_link_libraries(motor_run PUBLIC utilities libzmq-static)
target_link_directories(motor_run PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})

target_link_libraries(dyna_run PUBLIC utilities dynamixel_sdk libzmq-static)
target_link_directories(dyna_run PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})

target_link_libraries(vr_interface_run PUBLIC utilities libzmq-static vr_interface)
target_link_directories(vr_interface_run PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})

# Assuming you have custom macros
apply_global_cxx_flags_to_all_targets()
remove_flag_from_file(dyna_run src/dynamixel_run.cpp -Werror)
