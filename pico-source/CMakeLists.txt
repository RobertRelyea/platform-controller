cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

pico_sdk_init()

project(pico_platform_controller C CXX ASM)

# QuickPID source
add_subdirectory(./PicoQuickPID)

# Platform controller source
add_subdirectory(./src)
