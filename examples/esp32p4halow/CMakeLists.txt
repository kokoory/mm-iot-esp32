cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{MMIOT_ROOT})
    message(FATAL_ERROR "MMIOT_ROOT environment variable not defined. Please set as the path to the framework directory in the MM-IoT-SDK")
endif()

message(STATUS "MMIOT_ROOT: $ENV{MMIOT_ROOT}")

add_compile_definitions(LWIP_NETIF_LINK_CALLBACK=1)

if(DEFINED COUNTRY_CODE)
    message("Using country code: ${COUNTRY_CODE}")
    add_compile_definitions(COUNTRY_CODE="${COUNTRY_CODE}")
endif()

add_compile_options(-fdiagnostics-color=always -w)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_halow_drone)
