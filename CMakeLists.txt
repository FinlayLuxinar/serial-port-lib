cmake_minimum_required(VERSION 3.15)
project(serial-port-lib VERSION 1.0.0 LANGUAGES CXX)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create the library target
add_library(serial-port-lib
    src/serialport.cpp
)

# Add the include directory
target_include_directories(serial-port-lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optionally create an alias for easier use
add_library(serial-port-lib::serial-port-lib ALIAS serial-port-lib)

include(FetchContent)

FetchContent_Declare(
    serial-port-lib
    GIT_REPOSITORY https://github.com/your-username/serial-port-lib.git
    GIT_TAG        main # Or specify a specific tag/release
)
FetchContent_MakeAvailable(serial-port-lib)

# Add your application
add_executable(MyApp main.cpp)

# Link the serial-port-lib
target_link_libraries(MyApp PRIVATE serial-port-lib::serial-port-lib)
