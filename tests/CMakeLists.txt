include_directories (doctest)
include_directories (../)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

add_executable (tests main.cpp communication_tests.cpp time_out_tests.cpp pseudoterminal.cpp common.cpp)

target_link_libraries(tests PRIVATE lin_serial Threads::Threads)

target_compile_features(tests PRIVATE cxx_std_17)