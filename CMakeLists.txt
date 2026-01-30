# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(MLDH_MACROPAD C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# Generate a header from the PIO source

add_executable(MLDH_MACROPAD
    MLDH_MACROPAD.cpp
    usb_descriptors.c
    anim.cpp
    ws2812.cpp
)

pico_set_program_name(MLDH_MACROPAD "MLDH_MACROPAD")
pico_set_program_version(MLDH_MACROPAD "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(MLDH_MACROPAD 0)
pico_enable_stdio_usb(MLDH_MACROPAD 1)

# Add the standard library to the build
target_link_libraries(MLDH_MACROPAD
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(MLDH_MACROPAD PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_generate_pio_header(MLDH_MACROPAD ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# Add any user requested libraries
target_link_libraries(MLDH_MACROPAD 
        hardware_i2c
        hardware_flash
        hardware_sync
        hardware_gpio
        hardware_pio
        tinyusb_device
        tinyusb_board
        pico_multicore
        )

pico_add_extra_outputs(MLDH_MACROPAD)

