cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico2 CACHE STRING "Board type")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(PULSER_test_integ C CXX ASM)
pico_sdk_init()

add_executable(PULSER_test_integ main.c)
target_compile_options(PULSER_test_integ PRIVATE -Wall -Wextra -Werror)
target_link_libraries(PULSER_test_integ pico_stdlib)
# pico_add_extra_outputs intentionally omitted — ELF is the deliverable.
