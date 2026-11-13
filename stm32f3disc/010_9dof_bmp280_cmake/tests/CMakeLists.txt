cmake_minimum_required(VERSION 3.22)

# Host-only test tree: configured without the ARM toolchain file.
project(010_9dof_bmp280_cmake_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(unit_tests
    test_main.c
    test_crc16.c
    test_axis_map.c
    test_bmp280.c
    test_frame.c
    test_calibration.c
    stubs/bsp_uart_stub.c
    stubs/bsp_flash_stub.c
    stubs/bmp280_regs_stub.c
    ../App/Src/crc16.c
    ../App/Src/telemetry.c
    ../App/Src/board_axes.c
    ../App/Src/cal_store.c
    ../Fusion/Src/axis_map.c
    ../Fusion/Src/calib_math.c
    ../Sensors/vendor/bmp280/driver_bmp280.c
)

target_include_directories(unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/../App/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Bsp/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Sensors/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Sensors/vendor/bmp280
    ${CMAKE_CURRENT_SOURCE_DIR}/../Fusion/Inc
)

target_compile_definitions(unit_tests PRIVATE HOST_TEST=1)
target_compile_options(unit_tests PRIVATE
    -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wfloat-equal)

add_test(NAME unit COMMAND unit_tests)
