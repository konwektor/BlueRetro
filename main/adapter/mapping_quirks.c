/*
 * Copyright (c) 2021, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "zephyr/types.h"
#include "tools/util.h"
#include "adapter.h"
#include "mapping_quirks.h"

static void face_btns_invert(struct raw_src_mapping *map) {
    uint32_t tmp = map->btns_mask[PAD_RB_LEFT];

    map->btns_mask[PAD_RB_LEFT] = map->btns_mask[PAD_RB_UP];
    map->btns_mask[PAD_RB_UP] = tmp;

    tmp = map->btns_mask[PAD_RB_DOWN];
    map->btns_mask[PAD_RB_DOWN] = map->btns_mask[PAD_RB_RIGHT];
    map->btns_mask[PAD_RB_RIGHT] = tmp;
}

static void face_btns_rotate_right(struct raw_src_mapping *map) {
    uint32_t tmp = map->btns_mask[PAD_RB_UP];

    map->btns_mask[PAD_RB_UP] = map->btns_mask[PAD_RB_LEFT];
    map->btns_mask[PAD_RB_LEFT] = map->btns_mask[PAD_RB_DOWN];
    map->btns_mask[PAD_RB_DOWN] = map->btns_mask[PAD_RB_RIGHT];
    map->btns_mask[PAD_RB_RIGHT] = tmp;
}

static void face_btns_trigger_to_6buttons(struct raw_src_mapping *map) {
    map->desc[0] = 0x000000FF;

    map->btns_mask[PAD_LM] = map->btns_mask[PAD_RB_LEFT];
    map->btns_mask[PAD_RB_LEFT] = map->btns_mask[PAD_RB_DOWN];
    map->btns_mask[PAD_RB_DOWN] = map->btns_mask[PAD_RB_RIGHT];
    map->btns_mask[PAD_RB_RIGHT] = 0;

    map->axes_to_btns[TRIG_L] = PAD_RS;
    map->axes_to_btns[TRIG_R] = PAD_RB_RIGHT;
}

static void trigger_pri_sec_invert(struct raw_src_mapping *map) {
    uint32_t tmp = map->btns_mask[PAD_LM];

    map->btns_mask[PAD_LM] = map->btns_mask[PAD_LS];
    map->btns_mask[PAD_LS] = tmp;

    tmp = map->btns_mask[PAD_RM];
    map->btns_mask[PAD_RM] = map->btns_mask[PAD_RS];
    map->btns_mask[PAD_RS] = tmp;
}

static void sw_left_joycon(struct raw_src_mapping *map) {
    map->btns_mask[PAD_MM] = map->btns_mask[PAD_MQ];
    map->btns_mask[PAD_MQ] = 0;

    map->btns_mask[PAD_RM] = map->btns_mask[PAD_RT];
    map->btns_mask[PAD_RS] = map->btns_mask[PAD_LS];
    map->btns_mask[PAD_RT] = 0;

    map->btns_mask[PAD_LS] = map->btns_mask[PAD_LM];
    map->btns_mask[PAD_LM] = map->btns_mask[PAD_LT];
    map->btns_mask[PAD_LT] = 0;

    map->btns_mask[PAD_RB_UP] = map->btns_mask[PAD_LD_RIGHT];
    map->btns_mask[PAD_RB_LEFT] = map->btns_mask[PAD_LD_UP];
    map->btns_mask[PAD_RB_DOWN] = map->btns_mask[PAD_LD_LEFT];
    map->btns_mask[PAD_RB_RIGHT] = map->btns_mask[PAD_LD_DOWN];

    map->btns_mask[PAD_LD_LEFT] = 0;
    map->btns_mask[PAD_LD_DOWN] = 0;
    map->btns_mask[PAD_LD_RIGHT] = 0;
    map->btns_mask[PAD_LD_UP] = 0;
}

static void sw_right_joycon(struct raw_src_mapping *map) {
    map->btns_mask[PAD_MS] = map->btns_mask[PAD_MT];
    map->btns_mask[PAD_MT] = 0;

    map->btns_mask[PAD_LM] = map->btns_mask[PAD_LT];
    map->btns_mask[PAD_LS] = map->btns_mask[PAD_RS];
    map->btns_mask[PAD_LT] = 0;
    map->btns_mask[PAD_LJ] = map->btns_mask[PAD_RJ];

    map->btns_mask[PAD_RS] = map->btns_mask[PAD_RM];
    map->btns_mask[PAD_RM] = map->btns_mask[PAD_RT];
    map->btns_mask[PAD_RT] = 0;
    map->btns_mask[PAD_RJ] = 0;

    face_btns_rotate_right(map);
}

static void n64_8bitdo(struct raw_src_mapping *map) {
    map->mask[0] = 0x23150FFF;
    map->desc[0] = 0x0000000F;

    map->btns_mask[PAD_RY_UP] = BIT(8);
    map->btns_mask[PAD_RX_RIGHT] = BIT(9);
    map->btns_mask[PAD_RY_DOWN] = map->btns_mask[PAD_RB_LEFT];
    map->btns_mask[PAD_RX_LEFT] = map->btns_mask[PAD_RB_UP];
    map->btns_mask[PAD_LM] = map->btns_mask[PAD_MS];
    map->btns_mask[PAD_RB_LEFT] = map->btns_mask[PAD_RB_RIGHT];
    map->btns_mask[PAD_RB_RIGHT] = 0;
    map->btns_mask[PAD_RB_UP] = 0;
    map->btns_mask[PAD_MS] = 0;
    map->btns_mask[PAD_RT] = 0;
}

void mapping_quirks_apply(struct bt_data *bt_data) {
    if (atomic_test_bit(&bt_data->flags, BT_QUIRK_FACE_BTNS_INVERT)) {
        face_btns_invert(&bt_data->raw_src_mappings[PAD]);
    }
    if (atomic_test_bit(&bt_data->flags, BT_QUIRK_FACE_BTNS_ROTATE_RIGHT)) {
        face_btns_rotate_right(&bt_data->raw_src_mappings[PAD]);
    }
    if (atomic_test_bit(&bt_data->flags, BT_QUIRK_FACE_BTNS_TRIGGER_TO_6BUTTONS)) {
        face_btns_trigger_to_6buttons(&bt_data->raw_src_mappings[PAD]);
    }
    if (atomic_test_bit(&bt_data->flags, BT_QUIRK_TRIGGER_PRI_SEC_INVERT)) {
        trigger_pri_sec_invert(&bt_data->raw_src_mappings[PAD]);
    }
    if (atomic_test_bit(&bt_data->flags, BT_QUIRK_SW_LEFT_JOYCON)) {
        sw_left_joycon(&bt_data->raw_src_mappings[PAD]);
    }
    if (atomic_test_bit(&bt_data->flags, BT_QUIRK_SW_RIGHT_JOYCON)) {
        sw_right_joycon(&bt_data->raw_src_mappings[PAD]);
    }
    if (atomic_test_bit(&bt_data->flags, BT_QUIRK_8BITDO_N64)) {
        n64_8bitdo(&bt_data->raw_src_mappings[PAD]);
    }
}
