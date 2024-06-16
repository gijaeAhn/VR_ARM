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

# List of executables and their sources - corrected format
set(EXECUTABLES
        rmd_run          src/rmd_run.cpp
        motor_run        src/motor_run.cpp
        dyna_run         src/dynamixel_run.cpp
        vr_interface_run src/vr_interface_run.cpp
)

list(LENGTH EXECUTABLES EXECUTABLES_LENGTH)
math(EXPR LAST_INDEX "${EXECUTABLES_LENGTH} - 1")

foreach(EXEC_INDEX RANGE 0 ${LAST_INDEX} 2)
  list(GET EXECUTABLES ${EXEC_INDEX} EXEC_NAME)
  math(EXPR SRC_INDEX "${EXEC_INDEX} + 1")
  list(GET EXECUTABLES ${SRC_INDEX} SRC_FILE)

    add_executable(${EXEC_NAME} ${SRC_FILE})
    target_include_directories(${EXEC_NAME} PUBLIC ${ROOT_DIRECTORY} ${CMAKE_CURRENT_SOURCE_DIR})

    # Conditionally link libraries based on the executable
    if(EXEC_NAME STREQUAL "rmd_run")
        target_link_libraries(${EXEC_NAME} PUBLIC rmd_driver utilities libzmq-static)
        target_link_directories(${EXEC_NAME} PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})
    elseif(EXEC_NAME STREQUAL "motor_run")
        target_link_libraries(${EXEC_NAME} PUBLIC utilities libzmq-static)
        target_link_directories(${EXEC_NAME} PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})
    elseif(EXEC_NAME STREQUAL "dyna_run")
        target_link_libraries(${EXEC_NAME} PUBLIC utilities dynamixel_sdk libzmq-static)
        target_link_directories(${EXEC_NAME} PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})
    elseif(EXEC_NAME STREQUAL "vr_interface_run")
        target_link_libraries(${EXEC_NAME} PUBLIC utilities libzmq-static vr_interface)
        target_link_directories(${EXEC_NAME} PUBLIC ${math_INCLUDE_DIR} ${utilities_INCLUDE_DIR})
    endif()
endforeach()

# Custom macros
apply_global_cxx_flags_to_all_targets()
remove_flag_from_file(dyna_run src/dynamixel_run.cpp -Werror)
