cmake_minimum_required(VERSION 3.10)

project(LinSerial)

option(BUILD_EXAMPLE "Build example program" ON)
option(BUILD_TESTS "Build tests" ON)

#set(CMAKE_BUILD_TYPE Debug)

add_library(lin_serial STATIC LinSerial.cpp)

if(BUILD_EXAMPLE)
    add_executable(lin_serial_example Example.cpp)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(lin_serial_example PRIVATE lin_serial Threads::Threads)
endif(BUILD_EXAMPLE)

if (BUILD_TESTS)
    add_subdirectory (tests)
endif(BUILD_TESTS)
