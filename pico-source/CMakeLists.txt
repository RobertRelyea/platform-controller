cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
#include($ENV{PICO_SDK_PATH}/pico_sdk_import.cmake)
#include($ENV{PICO_SDK_PATH}/pico_extras_import_optional.cmake)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
# include($ENV{PICO_SDK_PATH}/example_auto_set_url.cmake)

pico_sdk_init()

project(pico_platform_controller C CXX ASM)

add_subdirectory(./src)
