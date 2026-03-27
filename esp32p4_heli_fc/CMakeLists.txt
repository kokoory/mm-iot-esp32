# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{MMIOT_ROOT})
    message(FATAL_ERROR "MMIOT_ROOT environment variable not defined. Please set as the path to the framework directory in the MM-IoT-SDK")
endif()

message(STATUS "MMIOT_ROOT: $ENV{MMIOT_ROOT}")

# Workaround: ESP-IDF < 6.1 does not expose LWIP_NETIF_LINK_CALLBACK via KConfig.
# https://github.com/espressif/esp-idf/commit/16b02aeb7a805de671ca92c0c7630eda091794ce
if ((IDF_VERSION_MAJOR VERSION_LESS 6) OR
    (IDF_VERSION_MAJOR EQUAL 6 AND IDF_VERSION_MINOR LESS_EQUAL 0))
    add_compile_definitions(LWIP_NETIF_LINK_CALLBACK=1)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_heli_fc)
