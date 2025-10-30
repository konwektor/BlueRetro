#ifndef _OGX360_H_
#define _OGX360_H_
#include "adapter/adapter.h"

//extern bool rumble_pending[4];

//void ogx360_acc_toggle_fb(uint32_t wired_id, uint32_t duration_us, uint16_t left_motor, uint16_t right_motor);
void ogx360_meta_init(struct wired_ctrl *ctrl_data);
void ogx360_init_buffer(int32_t dev_mode, struct wired_data *wired_data);
void ogx360_from_generic(int32_t dev_mode, struct wired_ctrl *ctrl_data, struct wired_data *wired_data);
void ogx360_fb_to_generic(int32_t dev_mode, struct raw_fb *raw_fb_data, struct generic_fb *fb_data);
//void trigger_special_rumble(int wired_index, int repetitions);


#endif /* _OGX360_H_ */
