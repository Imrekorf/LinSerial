if (NOT UNIX)
    message("FATAL: Building tests is only supported on unix")
    return()
endif()

if(NOT DEFINED SOCAT)
    message("FATAL: Needs socat to run tests - https://linux.die.net/man/1/socat")
    return()
endif()

include_directories (doctest)
include_directories (../)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

add_executable (tests main.cpp communication_tests.cpp time_out_tests.cpp pts.cpp helpers.cpp)

target_link_libraries(tests PRIVATE lin_serial Threads::Threads)