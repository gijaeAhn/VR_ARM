cmake_minimum_required(VERSION 3.23.0)
project(utilities)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)

add_library(utilities SHARED
            include/parser.hpp
            include/address.hpp
            include/timer.hpp
            include/debug.hpp
            include/param.hpp
            include/parser_yaml.hpp
            src/dummy.cpp
)

target_link_libraries(utilities Eigen3::Eigen)
target_include_directories(utilities PUBLIC ${ROOT_DIRECTORY})

