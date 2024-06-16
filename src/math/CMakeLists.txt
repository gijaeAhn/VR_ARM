cmake_minimum_required(VERSION 3.23.0)
project(math)

option(PYTHON_BINDINGS "Building Python bindings" OFF)
option(BUILD_TESTING "Build unit and integration tests" OFF)
option(SETUP_TEST_IFNAME "Set-up the test VCAN interface automatically" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


find_package(Eigen3 REQUIRED)

add_library(math
        armDynamcisSolver.cpp
        armKinematicsSolver.cpp
        transfrom.cpp
)

target_link_libraries(math Eigen3::Eigen)
target_include_directories(math PUBLIC ${ROOT_DIRECTORY})

if(BUILD_TESTING)
  enable_testing()
  # Add test configuration here
endif()

if(PYTHON_BINDINGS)
  # Add Python binding configuration here
endif()