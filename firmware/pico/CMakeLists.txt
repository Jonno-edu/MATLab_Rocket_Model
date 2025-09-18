cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pick board early for Pico W
set(PICO_BOARD pico2_w)

# Import the SDK via the local helper (copied into this repo)
include(pico_sdk_import.cmake)

project(pico_apps C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize SDK
pico_sdk_init()

# Add apps
add_subdirectory(apps/HIL)