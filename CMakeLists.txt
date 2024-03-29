cmake_minimum_required(VERSION 3.10)

project(LinSerial)

if (NOT UNIX)
    message("FATAL: Building is only supported on unix")
    return()
endif()

option(BUILD_EXAMPLE "Build example program" ON)
option(BUILD_TESTS "Build tests" ON)

#set(CMAKE_BUILD_TYPE Debug)

add_library(lin_serial STATIC LinSerial.cpp)

if(BUILD_EXAMPLE)
    add_executable(lin_serial_example Example.cpp)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    add_definitions(-DLINSER_LOG_LEVEL=7)
    target_link_libraries(lin_serial_example PRIVATE lin_serial Threads::Threads)
    target_compile_options(lin_serial_example PRIVATE -Wall -Wextra -Wpedantic)
endif(BUILD_EXAMPLE)

if (BUILD_TESTS)
    add_subdirectory (tests)
endif(BUILD_TESTS)
