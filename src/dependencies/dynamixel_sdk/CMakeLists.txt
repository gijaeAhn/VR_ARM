################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.16.2)
project(dynamixel_sdk
        LANGUAGES CXX)

add_compile_options(-std=c++11)



################################################################################
# Build
################################################################################
include_directories(
  include/dynamixel_sdk
)


add_library(dynamixel_sdk
  src/dynamixel_sdk/packet_handler.cpp
  src/dynamixel_sdk/protocol1_packet_handler.cpp
  src/dynamixel_sdk/protocol2_packet_handler.cpp
  src/dynamixel_sdk/group_sync_read.cpp
  src/dynamixel_sdk/group_sync_write.cpp
  src/dynamixel_sdk/group_bulk_read.cpp
  src/dynamixel_sdk/group_bulk_write.cpp
  src/dynamixel_sdk/port_handler.cpp
  src/dynamixel_sdk/port_handler_linux.cpp
)

apply_global_cxx_flags_to_all_targets()
remove_flag_from_file(dynamixel_sdk src/dynamixel_sdk/protocol1_packet_handler.cpp -Werror)
remove_flag_from_file(dynamixel_sdk src/dynamixel_sdk/protocol2_packet_handler.cpp -Werror)
remove_flag_from_file(dynamixel_sdk src/dynamixel_sdk/group_sync_read.cpp -Werror)
remove_flag_from_file(dynamixel_sdk src/dynamixel_sdk/group_sync_write.cpp -Werror)
remove_flag_from_file(dynamixel_sdk src/dynamixel_sdk/group_bulk_read.cpp -Werror)
remove_flag_from_file(dynamixel_sdk src/dynamixel_sdk/group_bulk_write.cpp -Werror)
remove_flag_from_file(dynamixel_sdk src/dynamixel_sdk/port_handler_linux.cpp -Werror)








