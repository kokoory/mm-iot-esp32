# All sources: FC (Core 0) + Communication (Core 1)
file(GLOB_RECURSE ALL_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
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
    "${CMAKE_CURRENT_SOURCE_DIR}/comm"
    "${CMAKE_CURRENT_SOURCE_DIR}/comm/mavlink"
)

idf_component_register(
    SRCS ${ALL_SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES driver spi_master i2c uart mcpwm esp_adc esp_timer nvs_flash
                  morselib mm_shims mmipal lwip esp_http_server mmregdb esp_event
                  esp_driver_cam esp_driver_isp esp_driver_jpeg esp_h264 esp_netif
)
