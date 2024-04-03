cmake_minimum_required(VERSION 3.16.3)
project(robot)

option(PYTHON_BINDINGS "Building Python bindings" OFF)
option(BUILD_TESTING "Build unit and integration tests" OFF)
option(SETUP_TEST_IFNAME "Set-up the test VCAN interface automatically" OFF)

if(NOT UNIX)
  message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)



add_executable(rmd_test src/rmd_test.cpp)

target_include_directories(rmd_test PUBLIC 
                          ${CMAKE_SOURCE_DIR}
                          ${rmd_driver_SOURCE_DIR}/include 
)

target_link_libraries(rmd_test PUBLIC
                      rmd_driver)  