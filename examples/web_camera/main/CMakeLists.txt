# Copyright 2024 Morse Micro
# SPDX-License-Identifier: Apache-2.0
set(src
    "src/mm_app_loadconfig.c"
    "src/mm_app_common.c"
    "src/web_camera.c")

set(inc
    "src")

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_REQUIRES morselib mm_shims mmipal lwip mmregdb
                                     esp_http_server esp_camera)
