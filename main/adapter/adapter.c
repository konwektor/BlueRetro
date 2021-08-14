/*
 * Copyright (c) 2019-2020, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <xtensa/hal.h>
#include <esp_timer.h>
#include "sdkconfig.h"
#include "queue_bss.h"
#include "zephyr/types.h"
#include "tools/util.h"
#include "config.h"
#include "adapter.h"
#include "npiso.h"
#include "cdi.h"
#include "genesis.h"
#include "pce.h"
#include "real.h"
#include "jag.h"
#include "pcfx.h"
#include "ps.h"
#include "saturn.h"
#include "jvs.h"
#include "n64.h"
#include "dc.h"
#include "gc.h"
#include "hid_generic.h"
#include "ps3.h"
#include "wii.h"
#include "ps4_ps5.h"
#include "xb1.h"
#include "sw.h"
#include "parallel_1p.h"
#include "parallel_2p.h"
#include "parallel_auto.h"

const uint32_t hat_to_ld_btns[16] = {
    BIT(PAD_LD_UP), BIT(PAD_LD_UP) | BIT(PAD_LD_RIGHT), BIT(PAD_LD_RIGHT), BIT(PAD_LD_DOWN) | BIT(PAD_LD_RIGHT),
    BIT(PAD_LD_DOWN), BIT(PAD_LD_DOWN) | BIT(PAD_LD_LEFT), BIT(PAD_LD_LEFT), BIT(PAD_LD_UP) | BIT(PAD_LD_LEFT),
};

const uint32_t generic_btns_mask[32] = {
    BIT(PAD_LX_LEFT), BIT(PAD_LX_RIGHT), BIT(PAD_LY_DOWN), BIT(PAD_LY_UP),
    BIT(PAD_RX_LEFT), BIT(PAD_RX_RIGHT), BIT(PAD_RY_DOWN), BIT(PAD_RY_UP),
    BIT(PAD_LD_LEFT), BIT(PAD_LD_RIGHT), BIT(PAD_LD_DOWN), BIT(PAD_LD_UP),
    BIT(PAD_RD_LEFT), BIT(PAD_RD_RIGHT), BIT(PAD_RD_DOWN), BIT(PAD_RD_UP),
    BIT(PAD_RB_LEFT), BIT(PAD_RB_RIGHT), BIT(PAD_RB_DOWN), BIT(PAD_RB_UP),
    BIT(PAD_MM), BIT(PAD_MS), BIT(PAD_MT), BIT(PAD_MQ),
    BIT(PAD_LM), BIT(PAD_LS), BIT(PAD_LT), BIT(PAD_LJ),
    BIT(PAD_RM), BIT(PAD_RS), BIT(PAD_RT), BIT(PAD_RJ),
};

static to_generic_t to_generic_func[BT_MAX] = {
    hid_to_generic, /* HID_GENERIC */
    ps3_to_generic, /* PS3_DS3 */
    wii_to_generic, /* WII_CORE */
    wiin_to_generic, /* WII_NUNCHUCK */
    wiic_to_generic, /* WII_CLASSIC */
    wiiu_to_generic, /* WIIU_PRO */
    ps4_ps5_to_generic, /* PS4_DS4 */
    xb1_to_generic, /* XB1_S */
    xb1_to_generic, /* XB1_ADAPTIVE */
    sw_to_generic, /* SW */
    ps4_ps5_to_generic, /* PS5_DS */
};

static fb_from_generic_t fb_from_generic_func[BT_MAX] = {
    NULL, /* HID_GENERIC */
    ps3_fb_from_generic, /* PS3_DS3 */
    wii_fb_from_generic, /* WII_CORE */
    wii_fb_from_generic, /* WII_NUNCHUCK */
    wii_fb_from_generic, /* WII_CLASSIC */
    wii_fb_from_generic, /* WIIU_PRO */
    ps4_fb_from_generic, /* PS4_DS4 */
    xb1_fb_from_generic, /* XB1_S */
    xb1_fb_from_generic, /* XB1_ADAPTIVE */
    sw_fb_from_generic, /* SW */
    ps5_fb_from_generic, /* PS5_DS4 */
};

static from_generic_t from_generic_func[WIRED_MAX] = {
    NULL, //para_auto_from_generic, /* WIRED_AUTO */
    para_1p_from_generic, /* PARALLEL_1P */
    para_2p_from_generic, /* PARALLEL_2P */
    npiso_from_generic, /* NES */
    pce_from_generic, /* PCE */
    genesis_from_generic, /* GENESIS */
    npiso_from_generic, /* SNES */
    cdi_from_generic, /* CDI */
    NULL, /* CD32 */
    real_from_generic, /* REAL_3DO */
    jag_from_generic, /* JAGUAR */
    ps_from_generic, /* PSX */
    saturn_from_generic, /* SATURN */
    pcfx_from_generic, /* PCFX */
    jvs_from_generic, /* JVS */
    n64_from_generic, /* N64 */
    dc_from_generic, /* DC */
    ps_from_generic, /* PS2 */
    gc_from_generic, /* GC */
    NULL, /* WII_EXT */
    NULL, /* EXP_BOARD */
};

static fb_to_generic_t fb_to_generic_func[WIRED_MAX] = {
    NULL, /* WIRED_AUTO */
    NULL, /* PARALLEL_1P */
    NULL, /* PARALLEL_2P */
    NULL, /* NES */
    NULL, /* PCE */
    NULL, /* GENESIS */
    NULL, /* SNES */
    NULL, /* CDI */
    NULL, /* CD32 */
    NULL, /* REAL_3DO */
    NULL, /* JAGUAR */
    ps_fb_to_generic, /* PSX */
    NULL, /* SATURN */
    NULL, /* PCFX */
    NULL, /* JVS */
    n64_fb_to_generic, /* N64 */
    dc_fb_to_generic, /* DC */
    ps_fb_to_generic, /* PS2 */
    gc_fb_to_generic, /* GC */
    NULL, /* WII_EXT */
    NULL, /* EXP_BOARD */
};

static meta_init_t meta_init_func[WIRED_MAX] = {
    NULL, //para_auto_meta_init, /* WIRED_AUTO */
    para_1p_meta_init, /* PARALLEL_1P */
    para_2p_meta_init, /* PARALLEL_2P */
    npiso_meta_init, /* NES */
    pce_meta_init, /* PCE */
    genesis_meta_init, /* GENESIS */
    npiso_meta_init, /* SNES */
    cdi_meta_init, /* CDI */
    NULL, /* CD32 */
    real_meta_init, /* REAL_3DO */
    jag_meta_init, /* JAGUAR */
    ps_meta_init, /* PSX */
    saturn_meta_init, /* SATURN */
    pcfx_meta_init, /* PCFX */
    jvs_meta_init, /* JVS */
    n64_meta_init, /* N64 */
    dc_meta_init, /* DC */
    ps_meta_init, /* PS2 */
    gc_meta_init, /* GC */
    NULL, /* WII_EXT */
    NULL, /* EXP_BOARD */
};

static DRAM_ATTR buffer_init_t buffer_init_func[WIRED_MAX] = {
    NULL, //para_auto_init_buffer, /* WIRED_AUTO */
    para_1p_init_buffer, /* PARALLEL_1P */
    para_2p_init_buffer, /* PARALLEL_2P */
    npiso_init_buffer, /* NES */
    pce_init_buffer, /* PCE */
    genesis_init_buffer, /* GENESIS */
    npiso_init_buffer, /* SNES */
    cdi_init_buffer, /* CDI */
    NULL, /* CD32 */
    real_init_buffer, /* REAL_3DO */
    jag_init_buffer, /* JAGUAR */
    ps_init_buffer, /* PSX */
    saturn_init_buffer, /* SATURN */
    pcfx_init_buffer, /* PCFX */
    jvs_init_buffer, /* JVS */
    n64_init_buffer, /* N64 */
    dc_init_buffer, /* DC */
    ps_init_buffer, /* PS2 */
    gc_init_buffer, /* GC */
    NULL, /* WII_EXT */
    NULL, /* EXP_BOARD */
};

struct generic_ctrl ctrl_input;
struct generic_ctrl ctrl_output[WIRED_MAX_DEV];
struct generic_fb fb_input;
struct bt_adapter bt_adapter = {0};
struct wired_adapter wired_adapter = {0};

static uint32_t btn_id_to_btn_idx(uint8_t btn_id) {
    if (btn_id < 32) {
        return 0;
    }
    else if (btn_id >= 32 && btn_id < 64) {
        return 1;
    }
    else if (btn_id >= 64 && btn_id < 96) {
        return 2;
    }
    else {
        return 3;
    }
}

static uint32_t adapter_map_from_axis(struct map_cfg * map_cfg) {
    uint32_t out_mask = BIT(map_cfg->dst_id);
    struct generic_ctrl *out = &ctrl_output[map_cfg->dst_id];
    uint8_t src = map_cfg->src_btn;
    uint8_t dst = map_cfg->dst_btn;
    uint32_t dst_mask = BIT(dst & 0x1F);
    uint32_t dst_btn_idx = btn_id_to_btn_idx(dst);
    uint32_t src_axis_idx = btn_id_to_axis(src);
    uint32_t dst_axis_idx = btn_id_to_axis(dst);

    if (src_axis_idx == AXIS_NONE) {
        return 0;
    }

    /* Check if mapping dst exist in output */
    if (dst_mask & out->mask[dst_btn_idx]) {
        int32_t abs_src_value = abs(ctrl_input.axes[src_axis_idx].value);
        int32_t src_sign = btn_sign(ctrl_input.axes[src_axis_idx].meta->polarity, src);
        int32_t sign_check = src_sign * ctrl_input.axes[src_axis_idx].value;

        /* Check if the srv value sign match the src mapping sign */
        if (sign_check >= 0) {
            /* Check if dst is an axis */
            if (dst_mask & out->desc[dst_btn_idx]) {
                /* Keep track of source axis type */
                out->axes[dst_axis_idx].relative = ctrl_input.axes[src_axis_idx].meta->relative;
                /* Dst is an axis */
                int32_t deadzone = (int32_t)(((float)map_cfg->perc_deadzone/10000) * ctrl_input.axes[src_axis_idx].meta->abs_max) + ctrl_input.axes[src_axis_idx].meta->deadzone;
                /* Check if axis over deadzone */
                if (abs_src_value > deadzone) {
                    int32_t value = abs_src_value - deadzone;
                    int32_t dst_sign = btn_sign(out->axes[dst_axis_idx].meta->polarity, dst);
                    float scale, fvalue;
                    switch (map_cfg->algo & 0xF) {
                        case LINEAR:
                            scale = ((float)out->axes[dst_axis_idx].meta->abs_max / (ctrl_input.axes[src_axis_idx].meta->abs_max - deadzone)) * (((float)map_cfg->perc_max)/100);
                            break;
                        default:
                            scale = ((float)map_cfg->perc_max)/100;
                            break;

                    }
                    fvalue = dst_sign * value * scale;
                    value = (int32_t)fvalue;

                    if (abs(value) > abs(out->axes[dst_axis_idx].value)) {
                        out->axes[dst_axis_idx].value = value;
                    }
                }
            }
            else {
                /* Dst is a button */
                int32_t threshold = (int32_t)(((float)map_cfg->perc_threshold/100) * ctrl_input.axes[src_axis_idx].meta->abs_max);
                /* Check if axis over threshold */
                if (abs_src_value > threshold) {
                    out->btns[dst_btn_idx].value |= dst_mask;
                }
            }
        }
        /* Flag this dst for update */
        out->map_mask[dst_btn_idx] |= dst_mask;
    }

    return out_mask;
}

static uint32_t adapter_map_from_btn(struct map_cfg * map_cfg, uint32_t src_mask, uint32_t src_btn_idx) {
    uint32_t out_mask = BIT(map_cfg->dst_id);
    struct generic_ctrl *out = &ctrl_output[map_cfg->dst_id];
    uint8_t dst = map_cfg->dst_btn;
    uint32_t dst_mask = BIT(dst & 0x1F);
    uint32_t dst_btn_idx = btn_id_to_btn_idx(dst);

    /* Check if mapping dst exist in output */
    if (dst_mask & out->mask[dst_btn_idx]) {
        /* Check if button pressed */
        if (ctrl_input.btns[src_btn_idx].value & src_mask) {
            /* Check if dst is an axis */
            if (dst_mask & out->desc[dst_btn_idx]) {
                /* Dst is an axis */
                uint32_t axis_id = btn_id_to_axis(dst);

                if (axis_id == AXIS_NONE) {
                    return 0;
                }
                else {
                    float fvalue = out->axes[axis_id].meta->abs_max
                                * btn_sign(out->axes[axis_id].meta->polarity, dst)
                                * (((float)map_cfg->perc_max)/100);
                    int32_t value = (int32_t)fvalue;

                    if (abs(value) > abs(out->axes[axis_id].value)) {
                        out->axes[axis_id].value = value;
                    }
                }
            }
            else {
                /* Dst is a button */
                out->btns[dst_btn_idx].value |= dst_mask;
            }
        }
        /* Flag this dst for update */
        out->map_mask[dst_btn_idx] |= dst_mask;
    }

    return out_mask;
}

static uint32_t adapter_mapping(struct in_cfg * in_cfg) {
    uint32_t out_mask = 0;

    for (uint32_t i = 0; i < in_cfg->map_size; i++) {
        uint8_t src = in_cfg->map_cfg[i].src_btn;
        uint32_t src_mask = BIT(src & 0x1F);
        uint32_t src_btn_idx = btn_id_to_btn_idx(src);

        /* Check if mapping src exist in input */
        if (src_mask & ctrl_input.mask[src_btn_idx]) {
            /* Check if src is an axis */
            if (src_mask & ctrl_input.desc[src_btn_idx]) {
                /* Src is an axis */
                out_mask |= adapter_map_from_axis(&in_cfg->map_cfg[i]);
            }
            else {
                /* Src is a button */
                out_mask |= adapter_map_from_btn(&in_cfg->map_cfg[i], src_mask, src_btn_idx);
            }
        }
    }
    return out_mask;
}

static void adapter_fb_stop_cb(void* arg) {
    uint8_t dev_id = (uint8_t)(uintptr_t)arg;

    /* Send 1 byte, system that require callback stop shall look for that */
    queue_bss_enqueue(wired_adapter.input_q_hdl, &dev_id, 1);
}

int32_t btn_id_to_axis(uint8_t btn_id) {
    switch (btn_id) {
        case PAD_LX_LEFT:
        case PAD_LX_RIGHT:
        case MOUSE_WX_LEFT:
        case MOUSE_WX_RIGHT:
            return AXIS_LX;
        case PAD_LY_DOWN:
        case PAD_LY_UP:
        case MOUSE_WY_DOWN:
        case MOUSE_WY_UP:
            return AXIS_LY;
        case PAD_RX_LEFT:
        case PAD_RX_RIGHT:
        //case MOUSE_X_LEFT:
        //case MOUSE_X_RIGHT:
            return AXIS_RX;
        case PAD_RY_DOWN:
        case PAD_RY_UP:
        //case MOUSE_Y_DOWN:
        //case MOUSE_Y_UP:
            return AXIS_RY;
        case PAD_LM:
            return TRIG_L;
        case PAD_RM:
            return TRIG_R;
    }
    return AXIS_NONE;
}

uint32_t axis_to_btn_mask(uint8_t axis) {
    switch (axis) {
        case AXIS_LX:
            return BIT(PAD_LX_LEFT) | BIT(PAD_LX_RIGHT) | BIT(MOUSE_WX_LEFT) | BIT(MOUSE_WX_RIGHT);
        case AXIS_LY:
            return BIT(PAD_LY_DOWN) | BIT(PAD_LY_UP) | BIT(MOUSE_WY_DOWN) | BIT(MOUSE_WY_UP);
        case AXIS_RX:
            return BIT(PAD_RX_LEFT) | BIT(PAD_RX_RIGHT); /* BIT(MOUSE_X_LEFT) | BIT(MOUSE_X_RIGHT) */
        case AXIS_RY:
            return BIT(PAD_RY_DOWN) | BIT(PAD_RY_UP); /* BIT(MOUSE_Y_DOWN) | BIT(MOUSE_Y_UP) */
        case TRIG_L:
            return BIT(PAD_LM);
        case TRIG_R:
            return BIT(PAD_RM);
    }
    return 0x00000000;
}

int8_t btn_sign(uint32_t polarity, uint8_t btn_id) {
    switch (btn_id) {
        case PAD_LX_RIGHT:
        case PAD_LY_UP:
        case PAD_RX_RIGHT:
        case PAD_RY_UP:
        case PAD_LM:
        case PAD_RM:
        case MOUSE_WX_RIGHT:
        case MOUSE_WY_UP:
        //case MOUSE_X_RIGHT:
        //case MOUSE_Y_UP:
            return polarity ? -1 : 1;
        case PAD_LX_LEFT:
        case PAD_LY_DOWN:
        case PAD_RY_DOWN:
        case PAD_RX_LEFT:
        case MOUSE_WX_LEFT:
        case MOUSE_WY_DOWN:
        //case MOUSE_X_LEFT:
        //case MOUSE_Y_DOWN:
            return polarity ? 1 : -1;
    }
    return 1;
}

void IRAM_ATTR adapter_init_buffer(uint8_t wired_id) {
    if (wired_adapter.system_id != WIRED_NONE && buffer_init_func[wired_adapter.system_id]) {
        wired_adapter.data[wired_id].index = wired_id;
        buffer_init_func[wired_adapter.system_id](config.out_cfg[wired_id].dev_mode, &wired_adapter.data[wired_id]);
    }
}

void adapter_bridge(struct bt_data *bt_data) {
    uint32_t out_mask = 0;

    if (bt_data->dev_id != BT_NONE && to_generic_func[bt_data->dev_type]) {
        if (to_generic_func[bt_data->dev_type](bt_data, &ctrl_input)) {
            /* Unsupported report */
            return;
        }

#ifdef CONFIG_BLUERETRO_ADAPTER_INPUT_DBG
        printf("LX: %s%08X%s, LY: %s%08X%s, RX: %s%08X%s, RY: %s%08X%s, LT: %s%08X%s, RT: %s%08X%s, BTNS: %s%08X%s, BTNS: %s%08X%s, BTNS: %s%08X%s, BTNS: %s%08X%s",
            BOLD, ctrl_input.axes[0].value, RESET, BOLD, ctrl_input.axes[1].value, RESET, BOLD, ctrl_input.axes[2].value, RESET, BOLD, ctrl_input.axes[3].value, RESET,
            BOLD, ctrl_input.axes[4].value, RESET, BOLD, ctrl_input.axes[5].value, RESET, BOLD, ctrl_input.btns[0].value, RESET, BOLD, ctrl_input.btns[1].value, RESET,
            BOLD, ctrl_input.btns[2].value, RESET, BOLD, ctrl_input.btns[3].value, RESET);
#ifdef CONFIG_BLUERETRO_ADAPTER_BTNS_DBG
        uint32_t b = ctrl_input.btns[0].value;
        printf(" %sDL%s %sDR%s %sDD%s %sDU%s %sBL%s %sBR%s %sBD%s %sBU%s %sMM%s %sMS%s %sMT%s %sMQ%s %sLM%s %sLS%s %sLT%s %sLJ%s %sRM%s %sRS%s %sRT%s %sRJ%s",
            (b & BIT(PAD_LD_LEFT)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_LD_RIGHT)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_LD_DOWN)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_LD_UP)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RB_LEFT)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RB_RIGHT)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RB_DOWN)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RB_UP)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_MM)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_MS)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_MT)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_MQ)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_LM)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_LS)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_LT)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_LJ)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RM)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RS)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RT)) ? GREEN : RESET, RESET,
            (b & BIT(PAD_RJ)) ? GREEN : RESET, RESET);
#endif
        printf("\n");
#else
        if (wired_adapter.system_id != WIRED_NONE && from_generic_func[wired_adapter.system_id]) {
            meta_init_func[wired_adapter.system_id](ctrl_output);

            out_mask = adapter_mapping(&config.in_cfg[bt_data->dev_id]);

#ifdef CONFIG_BLUERETRO_ADAPTER_INPUT_MAP_DBG
            printf("LX: %s%08X%s, LY: %s%08X%s, RX: %s%08X%s, RY: %s%08X%s, LT: %s%08X%s, RT: %s%08X%s, BTNS: %s%08X%s, BTNS: %s%08X%s, BTNS: %s%08X%s, BTNS: %s%08X%s",
                BOLD, ctrl_output[0].axes[0].value, RESET, BOLD, ctrl_output[0].axes[1].value, RESET, BOLD, ctrl_output[0].axes[2].value, RESET, BOLD, ctrl_output[0].axes[3].value, RESET,
                BOLD, ctrl_output[0].axes[4].value, RESET, BOLD, ctrl_output[0].axes[5].value, RESET, BOLD, ctrl_output[0].btns[0].value, RESET, BOLD, ctrl_output[0].btns[1].value, RESET,
                BOLD, ctrl_output[0].btns[2].value, RESET, BOLD, ctrl_output[0].btns[3].value, RESET);
#ifdef CONFIG_BLUERETRO_ADAPTER_BTNS_DBG
            uint32_t b = ctrl_output[0].btns[0].value;
            printf(" %sDL%s %sDR%s %sDD%s %sDU%s %sBL%s %sBR%s %sBD%s %sBU%s %sMM%s %sMS%s %sMT%s %sMQ%s %sLM%s %sLS%s %sLT%s %sLJ%s %sRM%s %sRS%s %sRT%s %sRJ%s",
                (b & BIT(PAD_LD_LEFT)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_LD_RIGHT)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_LD_DOWN)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_LD_UP)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RB_LEFT)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RB_RIGHT)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RB_DOWN)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RB_UP)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_MM)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_MS)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_MT)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_MQ)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_LM)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_LS)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_LT)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_LJ)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RM)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RS)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RT)) ? GREEN : RESET, RESET,
                (b & BIT(PAD_RJ)) ? GREEN : RESET, RESET);
#endif
            printf("\n");
#else
            for (uint32_t i = 0; out_mask; i++, out_mask >>= 1) {
                if (out_mask & 0x1) {
                    ctrl_output[i].index = i;
                    from_generic_func[wired_adapter.system_id](config.out_cfg[i].dev_mode, &ctrl_output[i], &wired_adapter.data[i]);
                }
            }
#endif
        }
#endif
    }
}

void adapter_fb_stop_timer_start(uint8_t dev_id, uint64_t dur_us) {
    if (wired_adapter.data[dev_id].fb_timer_hdl == NULL) {
        const esp_timer_create_args_t fb_timer_args = {
            .callback = &adapter_fb_stop_cb,
            .arg = (void*)(uintptr_t)dev_id,
            .name = "fb_timer"
        };
        esp_timer_create(&fb_timer_args, (esp_timer_handle_t *)&wired_adapter.data[dev_id].fb_timer_hdl);
        esp_timer_start_once(wired_adapter.data[dev_id].fb_timer_hdl, dur_us);
    }
}

void adapter_fb_stop_timer_stop(uint8_t dev_id) {
    esp_timer_delete(wired_adapter.data[dev_id].fb_timer_hdl);
    wired_adapter.data[dev_id].fb_timer_hdl = NULL;
}

uint32_t adapter_bridge_fb(uint8_t *fb_data, uint32_t fb_len, struct bt_data *bt_data) {
    uint32_t ret = 0;
    if (wired_adapter.system_id != WIRED_NONE && fb_to_generic_func[wired_adapter.system_id]) {
        fb_to_generic_func[wired_adapter.system_id](config.out_cfg[bt_data->dev_id].dev_mode, fb_data, fb_len, &fb_input);

        if (bt_data->dev_type != BT_NONE && fb_from_generic_func[bt_data->dev_type]) {
            fb_from_generic_func[bt_data->dev_type](&fb_input, bt_data);
            ret = 1;
        }
    }
    return ret;
}

void IRAM_ATTR adapter_q_fb(uint8_t *data, uint32_t len) {
    /* Best efford only on fb */
    queue_bss_enqueue(wired_adapter.input_q_hdl, data, len);
}

void adapter_init(void) {
    wired_adapter.system_id = WIRED_NONE;

    wired_adapter.input_q_hdl = queue_bss_init(16, 16);
    if (wired_adapter.input_q_hdl == NULL) {
        ets_printf("# %s: Failed to create fb queue\n", __FUNCTION__);
    }
}
