cmake_minimum_required(VERSION 3.20)
project(rmd_driver)


#SET PYTHON_BINDINGS OPTION ON If you want to integrate libraries with Python
option(PYTHON_BINDINGS "Building Python bindings" OFF)

if(NOT UNIX)
    message(FATAL_ERROR "Currently this driver only supports Linux!")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_TOOLCHAIN_FILE=${HOME}/vcpkg/scripts/buildsystems/vcpkg.cmake)




add_library(rmd_driver SHARED
        src/can/node.cpp
        src/can/utilities.cpp
        src/protocol/request.cpp
        src/protocol/response.cpp
        src/driver.cpp
        include/meta/motorParam.hpp
        include/meta/version.hpp
)

set(RMD_DRIVER_LIBRARIES "")
target_link_libraries(rmd_driver ${RMD_DRIVER_LIBRARIES})
target_include_directories(rmd_driver PUBLIC include)

if(PYTHON_BINDINGS)
    #Please Set Correct Pybind Directory
    set (pybind11_DIR /home/gj/.local/lib/python3.8/site-packages/pybind11/share/cmake/pybind11)
    find_package(Python COMPONENTS Interpreter Development)
    # Set pybind cmake directory
    find_package(pybind11 REQUIRED )
    pybind11_add_module(pymyactuator_rmd bindings/myactuator_rmd.cpp)
    target_link_libraries(pymyactuator_rmd PRIVATE rmd_driver)
    set_target_properties(pymyactuator_rmd PROPERTIES OUTPUT_NAME rmd_driver_py)

    install(TARGETS
            pymyactuator_rmd
            DESTINATION lib/${PROJECT_NAME}
    )
endif()