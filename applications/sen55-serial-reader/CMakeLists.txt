cmake_minimum_required(VERSION 3.10)
project(Sen55SerialReader VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# vcpkg integration
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif()
# set (VCPKG_DIR /home/fahmad/vcpkg)
# include(${VCPKG_DIR}/scripts/buildsystems/vcpkg.cmake)

# Add executable
add_executable(sen55-serial-reader main.cpp serial.cpp)

# Link libraries if needed
# target_link_libraries(sen55-serial-reader PRIVATE ...)