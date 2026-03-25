file(GLOB MAIN_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/mavlink/*.c"
)

idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}"
                 "${CMAKE_CURRENT_SOURCE_DIR}/mavlink"
    PRIV_REQUIRES driver morselib mm_shims mmipal lwip esp_timer esp_http_server mmregdb nvs_flash
)
