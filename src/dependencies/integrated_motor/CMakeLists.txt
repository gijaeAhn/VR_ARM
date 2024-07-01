cmake_minimum_required(VERSION 3.23.0)
project(integrated_driver)


#SET PYTHON_BINDINGS OPTION ON If you want to integrate libraries with Python
option(PYTHON_BINDINGS "Building Python bindings" OFF)

if(NOT PYTHON_BINDINGS)
    message("PY BIND HAS NOT SET")
endif()

if(NOT UNIX)
    message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(${ROOT_DIRECTORY})



add_library(integrated_driver SHARED src/integrated_driver.cpp
)

set(RMD_DRIVER_LIBRARIES "")
target_link_libraries(rmd_driver ${RMD_DRIVER_LIBRARIES})
target_include_directories(rmd_driver PUBLIC include)
