cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 010_9dof_bmp280_cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

# ---------------------------------------------------------------------------
# Build identity: git describe --always --dirty, refreshed on every build
# into gen/build_info.h (rewritten only when it changes).
# ---------------------------------------------------------------------------
find_package(Git QUIET)
set(BUILD_INFO_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
set(BUILD_INFO_H ${BUILD_INFO_DIR}/build_info.h)
add_custom_target(build_info
    COMMAND ${CMAKE_COMMAND}
            -DOUT=${BUILD_INFO_H}
            -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -DGIT_EXECUTABLE=${GIT_EXECUTABLE}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_info.cmake
    BYPRODUCTS ${BUILD_INFO_H}
    COMMENT "Refreshing build_info.h"
    VERBATIM)

# Size budgets (NFR-12 / NFR-13), enforced after every link.
set(FLASH_BUDGET 65536 CACHE STRING "Flash budget in bytes")
set(RAM_BUDGET 12288 CACHE STRING "RAM budget in bytes")

# ---------------------------------------------------------------------------
# Our code: Bsp, Sensors, Fusion, App
# ---------------------------------------------------------------------------
option(OUR_WERROR "Treat warnings in our sources as errors" OFF)

set(OUR_WARNINGS
    -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wfloat-equal
    -fno-common -ffunction-sections -fdata-sections)
if(OUR_WERROR)
    list(APPEND OUR_WARNINGS -Werror)
endif()

function(add_our_library name dir)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${dir}/Inc)
    target_link_libraries(${name} PUBLIC stm32cubemx)
    target_compile_options(${name} PRIVATE ${OUR_WARNINGS})
endfunction()

add_our_library(bsp Bsp
    Bsp/Src/bsp_time.c
    Bsp/Src/bsp_uart.c
    Bsp/Src/bsp_i2c.c
    Bsp/Src/bsp_spi.c
)
# printf retarget (__io_putchar) only in Debug.
target_compile_definitions(bsp PRIVATE $<$<CONFIG:Debug>:BSP_UART_STDIO>)

add_our_library(sensors Sensors
    Sensors/Src/i3g4250d.c
    Sensors/Src/lsm303agr.c
    Sensors/Src/bmp280.c
    Sensors/vendor/bmp280/driver_bmp280.c
)
target_include_directories(sensors PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Sensors/vendor/bmp280)
target_link_libraries(sensors PUBLIC bsp)
# LibDriver's bmp280_init(bmp280_handle_t *) collides with the wrapper API.
set_source_files_properties(Sensors/vendor/bmp280/driver_bmp280.c
    PROPERTIES COMPILE_DEFINITIONS "bmp280_init=ld_bmp280_init")

# Fusion: axis map now; Madgwick, baro filter and altitude arrive in M4.
add_our_library(fusion Fusion
    Fusion/Src/axis_map.c
)
target_link_libraries(fusion PUBLIC sensors)

add_our_library(app App
    App/Src/app.c
    App/Src/crc16.c
    App/Src/sched.c
    App/Src/telemetry.c
)
target_link_libraries(app PUBLIC bsp fusion)
target_include_directories(app PRIVATE ${BUILD_INFO_DIR})
add_dependencies(app build_info)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    app
)

# ---------------------------------------------------------------------------
# Artifacts (.hex/.bin) + size budget
# ---------------------------------------------------------------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/size_check.py
            --elf $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            --size-tool ${CMAKE_SIZE}
            --flash-max ${FLASH_BUDGET}
            --ram-max ${RAM_BUDGET}
    COMMENT "Generating .hex/.bin and checking the size budget"
    VERBATIM)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES
        ${CMAKE_PROJECT_NAME}.map
        ${CMAKE_PROJECT_NAME}.hex
        ${CMAKE_PROJECT_NAME}.bin)

# ---------------------------------------------------------------------------
# Flash target (OpenOCD, ST-LINK/V2-1 on the F3DISCOVERY)
# ---------------------------------------------------------------------------
find_program(OPENOCD_EXECUTABLE openocd)
if(OPENOCD_EXECUTABLE)
    add_custom_target(flash
        COMMAND ${OPENOCD_EXECUTABLE}
                -f interface/stlink.cfg -f target/stm32f3x.cfg
                -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Programming $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}> with OpenOCD"
        USES_TERMINAL)
else()
    message(WARNING "openocd not found: the 'flash' target is unavailable")
endif()
