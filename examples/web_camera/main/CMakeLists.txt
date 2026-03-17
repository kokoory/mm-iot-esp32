set(src
    "src/mm_app_loadconfig.c"
    "src/mm_app_common.c"
    "src/web_camera.c")

set(inc
    "src")

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_REQUIRES driver morselib mm_shims mmipal lwip esp_timer esp_http_server)
