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



find_package(ZeroMQ REQUIRED)




#RMD TEST

add_executable(rmd_test       src/rmd_test.cpp)
target_include_directories(rmd_test PUBLIC 
                          ${CMAKE_SOURCE_DIR}
                          ${rmd_driver_SOURCE_DIR}/include 
)

target_link_libraries(rmd_test PUBLIC
                      rmd_driver)  



#DYNAMIXEL TEST
# add_executable(dynamixel_test src/dynamixel_test.cpp)
# target_include_directories(dynamixel_test PUBLIC
#                           ${ROOT_DIRECTORY}
#                           ${dynamixel_sdk_SOURCE_DIR}/include )
                          
# target_link_libraries(dynamixel_test PUBLIC
#                       dynamixel_sdk)

add_executable(motor_test src/motor_test.cpp)
target_include_directories(motor_test  PUBLIC
                          ${CMAKE_SOURCE_DIR}
                          ${rmd_driver_SOURCE_DIR}/include)

target_link_libraries(motor_test PUBLIC
                      libzmq-static)  




add_executable(zmq_1 src/zmq_test1.cpp)
target_include_directories(zmq_1 PUBLIC
                          ${ROOT_DIRECTORY}
)


add_executable(zmq_2 src/zmq_test2.cpp)
target_include_directories(zmq_2 PUBLIC
                          ${ROOT_DIRECTORY}
)


target_link_libraries(zmq_1 libzmq-static)
target_link_libraries(zmq_2 libzmq-static)



