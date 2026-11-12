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
set(CMAKE_C_EXTENSIONS ON)


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

set(OUR_WARNINGS -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wfloat-equal)

add_library(sensors STATIC
    Sensors/BMP280/driver_bmp280.c
    Sensors/i3g4250d/i3g4250d.c
    Sensors/lsm303agr/lsm303agr.c
)
target_include_directories(sensors PUBLIC
    Sensors/BMP280
    Sensors/i3g4250d
    Sensors/lsm303agr
    Sensors/Common
)
target_compile_options(sensors PRIVATE ${OUR_WARNINGS})

add_library(bsp STATIC
    Bsp/stm32f3_discovery.c
    Bsp/stm32f3_discovery_accelerometer.c
    Bsp/stm32f3_discovery_gyroscope.c
)
target_include_directories(bsp PUBLIC Bsp)
target_link_libraries(bsp PUBLIC stm32cubemx sensors)
target_link_libraries(sensors PUBLIC bsp)
target_compile_options(bsp PRIVATE ${OUR_WARNINGS})

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

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    sensors
)
