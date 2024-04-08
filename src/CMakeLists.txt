cmake_minimum_required(VERSION 3.16.3)
project(VR_ARM
        VERSION 1.0
        LANGUAGES CXX)


set(CMAKE_CXX_STANDARD  17)
set(CMAKE_CXX_FLAGS  -pthread)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


set(ROOT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(utilities)
add_subdirectory(dependencies/dynamixel_sdk)
add_subdirectory(dependencies/rmd_driver)
add_subdirectory(motor_interface)
add_subdirectory(ros_interface)
add_subdirectory(vr_interface)
add_subdirectory(robot)

add_subdirectory(test)
add_subdirectory(run)



