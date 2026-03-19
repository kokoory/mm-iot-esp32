# Flight Controller sources (Core 0)
file(GLOB_RECURSE FC_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

# HaLow Communication sources (Core 1) - exclude duplicate RPC files
set(HALOW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../esp32p4halow/main")
file(GLOB HALOW_SRCS
    "${HALOW_DIR}/*.c"
    "${HALOW_DIR}/mavlink/*.c"
)
# Note: HaLow's rpc/*.c are intentionally excluded - we use the FC's rpc/ files

set(ALL_SRCS ${FC_SRCS} ${HALOW_SRCS})

set(INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/common"
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/agents"
    "${CMAKE_CURRENT_SOURCE_DIR}/control"
    "${CMAKE_CURRENT_SOURCE_DIR}/estimator"
    "${CMAKE_CURRENT_SOURCE_DIR}/uorb"
    "${CMAKE_CURRENT_SOURCE_DIR}/rpc"
    "${HALOW_DIR}"
    "${HALOW_DIR}/mavlink"
)

idf_component_register(
    SRCS ${ALL_SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES driver spi_master i2c uart mcpwm esp_adc esp_timer nvs_flash
                  morselib mm_shims
)
