# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{MMIOT_ROOT})
    message(FATAL_ERROR "MMIOT_ROOT environment variable not defined. Please set as the path to the framework directory in the MM-IoT-SDK")
endif()

message(STATUS "MMIOT_ROOT: $ENV{MMIOT_ROOT}")

# Required by mmipal for lwip netif callbacks
add_compile_definitions(LWIP_NETIF_LINK_CALLBACK=1)

# Suppress warnings from framework code (matches reference esp32p4halow)
add_compile_options(-fdiagnostics-color=always -w)

if(DEFINED COUNTRY_CODE)
    message("Using country code: ${COUNTRY_CODE}")
    add_compile_definitions(COUNTRY_CODE="${COUNTRY_CODE}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_heli_fc)

# Patch UVC driver to add Y16 format support for PureThermal thermal camera
execute_process(
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/patch_uvc_y16.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
