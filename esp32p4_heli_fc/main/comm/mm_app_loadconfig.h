/*
 * Copyright 2021-2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mmipal.h"
#include "mmwlan.h"

#ifdef __cplusplus
extern "C" {
#endif

const struct mmwlan_s1g_channel_list* load_channel_list(void);
void load_mmipal_init_args(struct mmipal_init_args *args);
void load_mmwlan_sta_args(struct mmwlan_sta_args *sta_config);
void load_mmwlan_settings(void);

#ifdef __cplusplus
}
#endif
