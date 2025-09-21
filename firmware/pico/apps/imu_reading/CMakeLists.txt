# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(blink_any C CXX ASM)

# Add executable. This MUST come before you modify the target.
add_executable(blink_any
    main.c # <-- Make sure your C file is named main.c, or change this to match
)

# This new line enables the automatic reset feature for picotool
target_compile_definitions(blink_any PRIVATE PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# --- Main Change Here ---
# Pull in all common dependencies in a single command
if (PICO_CYW43_SUPPORTED)
    target_link_libraries(blink_any pico_stdlib hardware_i2c pico_cyw43_arch_none)
else()
    target_link_libraries(blink_any pico_stdlib hardware_i2c)
endif()

# These new lines enable stdio (like printf) over USB and disable it for UART
pico_enable_stdio_usb(blink_any 1)
pico_enable_stdio_uart(blink_any 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(blink_any)

# add url via pico_set_program_url
