if(NOT DEFINED SOCAT)
    message("WARNING: Could not determine whether socat installed or not.\nSocat is needed for the test suite - https://linux.die.net/man/1/socat")
endif()

include_directories (doctest)
include_directories (../)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

add_executable (tests main.cpp communication_tests.cpp time_out_tests.cpp pseudoterminal.cpp helpers.cpp)

target_link_libraries(tests PRIVATE lin_serial Threads::Threads)