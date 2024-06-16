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

find_package(ZeroMQ REQUIRED)

# Common settings for all executables
set(COMMON_INCLUDE_DIRS ${ROOT_DIRECTORY})

# List of executables and their sources
set(EXECUTABLES
        rmd_test src/rmd_test.cpp
        motor_test src/motor_test.cpp
        dynamixel_test src/dynamixel_test.cpp
        zmq_1 src/zmq_test1.cpp
        zmq_2 src/zmq_test2.cpp
)

# Create each executable and set common properties
foreach(EXEC_NAME SRC_FILE IN LISTS EXECUTABLES)
  add_executable(${EXEC_NAME} ${SRC_FILE})
  target_include_directories(${EXEC_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})
endforeach()

# Specific target link libraries
target_link_libraries(rmd_test PUBLIC rmd_driver)
target_link_libraries(motor_test PUBLIC libzmq-static)
target_link_libraries(dynamixel_test PUBLIC dynamixel_sdk)
target_link_libraries(zmq_1 PUBLIC libzmq-static)
target_link_libraries(zmq_2 PUBLIC libzmq-static)