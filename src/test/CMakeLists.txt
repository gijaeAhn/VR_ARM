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

set(EXECUTABLES
    rmd_test src/rmd_test.cpp
    motor_test src/motor_test.cpp
    dynamixel_test src/dynamixel_test.cpp
    zmq_1 src/zmq_test1.cpp
    zmq_2 src/zmq_test2.cpp
)

# Calculating the length of EXECUTABLES
list(LENGTH EXECUTABLES EXECUTABLES_LENGTH)
math(EXPR LAST_INDEX "${EXECUTABLES_LENGTH} - 1")

# Correcting the FOREACH loop
foreach(EXEC_INDEX RANGE 0 ${LAST_INDEX} 2)
  list(GET EXECUTABLES ${EXEC_INDEX} EXEC_NAME)
  math(EXPR SRC_INDEX "${EXEC_INDEX} + 1")
  list(GET EXECUTABLES ${SRC_INDEX} SRC_FILE)
  
  add_executable(${EXEC_NAME} ${SRC_FILE})
  target_include_directories(${EXEC_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})

  # Configure linking specific to each executable
  if(EXEC_NAME STREQUAL "rmd_test")
    target_link_libraries(${EXEC_NAME} PUBLIC rmd_driver)
  elseif(EXEC_NAME STREQUAL "motor_test" OR EXEC_NAME STREQUAL "zmq_1" OR EXEC_NAME STREQUAL "zmq_2")
    target_link_libraries(${EXEC_NAME} PUBLIC libzmq-static)
  elseif(EXEC_NAME STREQUAL "dynamixel_test")
    target_link_libraries(${EXEC_NAME} PUBLIC dynamixel_sdk)
  endif()
endforeach()
