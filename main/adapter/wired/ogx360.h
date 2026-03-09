#ifndef _OGX360_H_
#define _OGX360_H_
#include "adapter/adapter.h"

void ogx360_meta_init(struct wired_ctrl *ctrl_data);
void ogx360_init_buffer(int32_t dev_mode, struct wired_data *wired_data);
void ogx360_from_generic(int32_t dev_mode, struct wired_ctrl *ctrl_data, struct wired_data *wired_data);
void ogx360_fb_to_generic(int32_t dev_mode, struct raw_fb *raw_fb_data, struct generic_fb *fb_data);

#endif /* _OGX360_H_ */
