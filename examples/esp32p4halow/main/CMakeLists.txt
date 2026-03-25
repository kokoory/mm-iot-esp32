set(src
    "src/mm_app_loadconfig.c"
    "src/mm_app_common.c"
    "src/camera_h264.c"
    "src/mavlink_uart.c"
    "src/esp32p4_halow_main.c")

set(inc
    "src")

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_REQUIRES driver morselib mm_shims mmipal lwip esp_timer esp_http_server mmregdb
                                     esp_driver_cam esp_driver_isp esp_driver_jpeg esp_h264)
