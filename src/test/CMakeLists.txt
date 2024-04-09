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


find_package(cppzmq CONFIG REQUIRED)








#RMD TEST

add_executable(rmd_test       src/rmd_test.cpp)
target_include_directories(rmd_test PUBLIC 
                          ${CMAKE_SOURCE_DIR}
                          ${rmd_driver_SOURCE_DIR}/include 
)

target_link_libraries(rmd_test PUBLIC
                      rmd_driver)  



#DYNAMIXEL TEST
add_executable(dynamixel_test src/dynamixel_test.cpp)
target_include_directories(dynamixel_test PUBLIC
                          ${ROOT_DIRECTORY}
                          ${dynamixel_sdk_SOURCE_DIR}/include )
                          
target_link_libraries(dynamixel_test PUBLIC
                      dynamixel_sdk)




add_executable(shm_1 src/shm_test1.cpp)
target_include_directories(shm_1 PUBLIC
                          ${ROOT_DIRECTORY}
)

target_include_directories(shm_1 PUBLIC ${ZeroMQ_INCLUDE_DIR})

                          

add_executable(shm_2 src/shm_test2.cpp)
target_include_directories(shm_2 PUBLIC
                          ${ROOT_DIRECTORY}
)
target_include_directories(shm_2 PUBLIC ${ZeroMQ_INCLUDE_DIR})



add_executable(shm_3 src/shm_test3.cpp)
target_include_directories(shm_3 PUBLIC
                          ${ROOT_DIRECTORY}
)

target_link_libraries(shm_1 PRIVATE cppzmq cppzmq-static)
target_link_libraries(shm_2 PRIVATE cppzmq cppzmq-static)



