# Copyright 2025 Morse Micro
# SPDX-License-Identifier: Apache-2.0
set(src
    "src/ap_sta_switch.c")

set(inc
    "src")

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_REQUIRES morselib mm_shims lwip mmregdb)
