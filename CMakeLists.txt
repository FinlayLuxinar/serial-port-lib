cmake_minimum_required(VERSION 3.10)
project(serial-port-lib VERSION 1.0.0 LANGUAGES CXX)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the library
add_library(serial-port-lib
    src/serialPort.cpp
)

# Add include directories so the library's headers are accessible
target_include_directories(serial-port-lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Define alias for convenience
add_library(serial-port-lib::serial-port-lib ALIAS serial-port-lib)
