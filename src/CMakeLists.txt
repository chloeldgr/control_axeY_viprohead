cmake_minimum_required(VERSION 3.10)
project(rt_text_periodic_task C)

set(CMAKE_C_STANDARD 11)

# Use pthread
find_package(Threads REQUIRED)

add_executable(rt_example
    src/rtController.c
)

target_link_libraries(rt_example
    Threads::Threads
)

# Enable compile flags useful for RT
target_compile_options(rt_example PRIVATE -Wall -Wextra -O2)
