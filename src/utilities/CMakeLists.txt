cmake_minimum_required(VERSION 3.23.0)
project(utilities)
        

if(NOT UNIX)
  message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


add_library(utilities SHARED
            include/parser.hpp
            include/address.hpp
            include/timer.hpp
            src/temp.cpp
)

target_compile_features(utilities PRIVATE cxx_std_17)
