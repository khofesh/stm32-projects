cmake_minimum_required(VERSION 3.22)

# Host-only test tree: configured without the ARM toolchain file.
project(010_stm32l4_spi_master_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(unit_tests
    test_main.c
    test_frame_check.c
    ../App/Src/frame_check.c
)
target_include_directories(unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../App/Inc)
target_compile_definitions(unit_tests PRIVATE HOST_TEST=1)
target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wfloat-equal)

add_test(NAME unit COMMAND unit_tests)
