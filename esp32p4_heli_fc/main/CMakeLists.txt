file(GLOB_RECURSE MAIN_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/agents/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/control/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/estimator/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/uorb/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/rpc/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/*.c"
)

set(INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/common"
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/agents"
    "${CMAKE_CURRENT_SOURCE_DIR}/control"
    "${CMAKE_CURRENT_SOURCE_DIR}/estimator"
    "${CMAKE_CURRENT_SOURCE_DIR}/uorb"
    "${CMAKE_CURRENT_SOURCE_DIR}/rpc"
)

idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES driver spi_master i2c uart mcpwm esp_adc esp_timer nvs_flash
)
