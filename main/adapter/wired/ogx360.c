#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <esp_timer.h>
#include "bluetooth/host.h"
#include "adapter/wireless/wireless.h"
#include "wired/ogx360_i2c.h" 
#include <esp32/rom/ets_sys.h>
#include "zephyr/types.h"
#include "tools/util.h"
#include "adapter/adapter.h"
#include "adapter/config.h"
#include "adapter/kb_monitor.h"
#include "adapter/wired/wired.h"
#include "adapter/wireless/wireless.h"
#include "ogx360.h"

//#define BIT(x) (1<<x)

#define BUTTON_MASK_SIZE 32
#define NUM_DIGITAL_BUTTONS 8
#define NUM_ANALOG_BUTTONS 6
#define NUM_8BIT_AXIS 2
#define NUM_16BIT_AXIS 4

bool pressed;
uint8_t axes_index;

enum { // Digital buttons
    OGX360_D_UP,
    OGX360_D_DOWN,
    OGX360_D_LEFT,
    OGX360_D_RIGHT,
    OGX360_START,
    OGX360_BACK,
    OGX360_LSTICK,
    OGX360_RSTICK
};

enum { // Analog buttons
    OGX360_A,
    OGX360_B,
    OGX360_X,
    OGX360_Y,
    OGX360_BLACK,
    OGX360_WHITE,
};

static DRAM_ATTR const struct ctrl_meta ogx360_axes_meta[ADAPTER_MAX_AXES] =
{
    {.size_min = -32768, .size_max = 32767, .neutral = 0x00, .abs_max = 0x7FFF, .abs_min = 0x8000},
    {.size_min = -32768, .size_max = 32767, .neutral = 0x00, .abs_max = 0x7FFF, .abs_min = 0x8000},
    {.size_min = -32768, .size_max = 32767, .neutral = 0x00, .abs_max = 0x7FFF, .abs_min = 0x8000},
    {.size_min = -32768, .size_max = 32767, .neutral = 0x00, .abs_max = 0x7FFF, .abs_min = 0x8000},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 255, .abs_min = 0x00},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 255, .abs_min = 0x00},
};


typedef struct __attribute__((packed)) usbd_duke_out
{
    uint8_t  controllerType; // 0xF1
    uint8_t  startByte;      // 0x00
    uint8_t  bLength;        // 0x06
    uint16_t wButtons;
    uint8_t  analogButtons[NUM_ANALOG_BUTTONS];
    uint8_t  axis8[NUM_8BIT_AXIS];
    uint16_t axis16[NUM_16BIT_AXIS];
} usbd_duke_out_t;

static DRAM_ATTR const uint8_t ogx360_axes_idx[ADAPTER_MAX_AXES] =
{
/*  AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, TRIG_L, TRIG_R  */
    0,       1,       2,       3,       4,      5
};


static const uint32_t ogx360_mask[4] = {0xBBFF0FFF, 0x00000000, 0x00000000, BR_COMBO_MASK};
static const uint32_t ogx360_desc[4] = {0x110000FF, 0x00000000, 0x00000000, 0x00000000};


static DRAM_ATTR const int ogx360_digital_btns_mask[BUTTON_MASK_SIZE] = {
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    OGX360_D_LEFT, OGX360_D_RIGHT, OGX360_D_DOWN, OGX360_D_UP,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    OGX360_START, OGX360_BACK, -1, -1,
    -1, -1, -1, OGX360_LSTICK,
    -1, -1, -1, OGX360_RSTICK
};

static DRAM_ATTR const int ogx360_analog_btns_mask[BUTTON_MASK_SIZE] = {
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    OGX360_X, OGX360_B, OGX360_A, OGX360_Y,
    -1, -1, -1, -1,
    -1, OGX360_BLACK, -1, -1,
    -1, OGX360_WHITE, -1, -1,
};



void IRAM_ATTR ogx360_init_buffer(int32_t dev_mode, struct wired_data *wired_data) {
     switch (dev_mode) {
        default:
        {
            struct usbd_duke_out *duke_out = (struct usbd_duke_out *)wired_data->output;
            struct usbd_duke_out *duke_out_mask = (struct usbd_duke_out *)wired_data->output_mask;

            memset((void *)duke_out, 0, sizeof(*duke_out));
            duke_out->controllerType = 0xF1;
            duke_out->startByte = 0x00;
            duke_out->bLength = 0x14;
            duke_out->wButtons = 0xFFFF;  // All buttons unpressed
            // Initialize analog buttons
            for (uint32_t i = 0; i < NUM_ANALOG_BUTTONS; i++) {
                duke_out->analogButtons[i] = 0x00;  // Default unpressed value
            }
            //Initialize 8-bit axes to their neutral positions
            for (uint32_t i = 0; i < NUM_8BIT_AXIS; i++) {
                duke_out->axis8[i] = ogx360_axes_meta[NUM_16BIT_AXIS + i].neutral; // Set 8-bit axes to neutral
            }
            // Initialize 16-bit axes to their neutral positions
            for (uint32_t i = 0; i < NUM_16BIT_AXIS; i++) {
                duke_out->axis16[ogx360_axes_idx[i]] = ogx360_axes_meta[i].neutral; // Set 16-bit axes to neutral
            }

            
            // Initialize mask structure
            duke_out_mask->controllerType = 0x00;
            duke_out_mask->startByte = 0x00;
            duke_out_mask->bLength = 0x00;
            duke_out_mask->wButtons = 0x0000;  // No buttons masked
            // Initialize analog buttons mask
            for (uint32_t i = 0; i < NUM_ANALOG_BUTTONS; i++) {
                duke_out_mask->analogButtons[i] = 0x00;  // Default unpressed value
            }
            // Initialize 8-bit axes mask
            for (uint32_t i = 0; i < NUM_8BIT_AXIS; i++) {
                duke_out_mask->axis8[i] = 0x00;  // No mask
            }
            // Initialize 16-bit axes mask
            for (uint32_t i = 0; i < NUM_16BIT_AXIS; i++) {
                duke_out_mask->axis16[i] = 0x0000;  // No mask
            }
            memset(wired_data->output_mask, 0x00, sizeof(struct usbd_duke_out));
            break;
        }
    }
}

void ogx360_meta_init(struct wired_ctrl *ctrl_data) {
    memset((void *)ctrl_data, 0, sizeof(*ctrl_data)*4);

    for (uint32_t i = 0; i < WIRED_MAX_DEV; i++) {
        for (uint32_t j = 0; j < ADAPTER_MAX_AXES; j++) {
            switch (config.out_cfg[i].dev_mode) {
                case DEV_PAD_ALT:
                    ctrl_data[i].mask = ogx360_mask;
                    ctrl_data[i].desc = ogx360_desc;
                    ctrl_data[i].axes[j].meta = &ogx360_axes_meta[j];
                    break;
                default:
                    ctrl_data[i].mask = ogx360_mask;
                    ctrl_data[i].desc = ogx360_desc;
                    ctrl_data[i].axes[j].meta = &ogx360_axes_meta[j];
                    break;
            }
        }
    }
}

void ogx360_ctrl_special_action(struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {

    if (ctrl_data->map_mask[0] & generic_btns_mask[PAD_MT]) {
    // printf("MT button detected\n"); // Debug

        if (ctrl_data->btns[0].value & generic_btns_mask[PAD_MT]) {
         //printf("MT button pressed\n"); // for Debug - to be sure if its working
            if (!atomic_test_bit(&wired_data->flags, WIRED_WAITING_FOR_RELEASE)) {
                atomic_set_bit(&wired_data->flags, WIRED_WAITING_FOR_RELEASE);
               // printf("Waiting for MT release\n"); // Debug to see if pushed and holded button is really recognized 
            }
        } 
        else {
            if (atomic_test_bit(&wired_data->flags, WIRED_WAITING_FOR_RELEASE)) {
                atomic_clear_bit(&wired_data->flags, WIRED_WAITING_FOR_RELEASE);
               // printf("MT button released\n"); // Debug
                //short rumble - call rumble for n-times loops - port number count  
                int repeat_count = wired_data->index + 1;  // Określ liczbę cykli
                start_rumble_sequence(ctrl_data->index, 500000, repeat_count);
              //  printf(" rumble activated\n"); // Debug
            }
        }
    }
}

void ogx360_ctrl_from_generic(struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {    
      // Ensure wired_data and wired_data->output are not NULL
    if (wired_data == NULL) {

        ets_printf("OGX360_CTRL_FROM_GENERIC : Error, NULL pointer!!!\n");
        return;
    }
    struct usbd_duke_out duke_out;
    
    memcpy((void *)&duke_out, wired_data->output, sizeof(duke_out));
    
    //Start output
    duke_out.controllerType = 0xF1;
    duke_out.startByte = 0;
    duke_out.bLength = ( sizeof(struct usbd_duke_out) + 3 ) / 4; //Number of 4-byte blocks sent to Xbox //0x14;
    
    duke_out.wButtons = 0;  // Digital buttons
    for (int i=0;i<BUTTON_MASK_SIZE;i++)
    {
        if (ogx360_digital_btns_mask[i] != -1)
        {
            if ((ctrl_data->btns[0].value & BIT(i)) != 0)
            {
                duke_out.wButtons |= BIT(ogx360_digital_btns_mask[i]);
            }   
        }
    }
    
    for (int i=0;i<BUTTON_MASK_SIZE;i++) // Analog buttons
    {
        if (ogx360_analog_btns_mask[i] != -1)
        {
            bool pressed = (ctrl_data->btns[0].value & BIT(i)) > 0;
            if (pressed)
            {
                duke_out.analogButtons[ogx360_analog_btns_mask[i]] = 0xFF;
            }
            else
            {
                duke_out.analogButtons[ogx360_analog_btns_mask[i]] = 0x00;
            }
        }
    }
    ogx360_ctrl_special_action(ctrl_data, wired_data);
  
    
    
    for (int i=0;i<NUM_16BIT_AXIS;i++) // 16 bit axis
    {
        if (ctrl_data->axes[i].value > ctrl_data->axes[i].meta->size_max) {
            duke_out.axis16[i] = ctrl_data->axes[i].meta->size_max;
        }
        else if (ctrl_data->axes[i].value < ctrl_data->axes[i].meta->size_min) {
            duke_out.axis16[i] = ctrl_data->axes[i].meta->size_min;
        }
        else {
            duke_out.axis16[i] = ctrl_data->axes[i].value;
        }
    }
    
    for (int i=0;i<NUM_8BIT_AXIS;i++) // 8 bit axis
    {
        uint8_t axes_index = NUM_16BIT_AXIS + i;
        if (ctrl_data->axes[axes_index].value > ctrl_data->axes[axes_index].meta->size_max) {
            duke_out.axis8[i] = ctrl_data->axes[axes_index].meta->size_max;
        }
        else if (ctrl_data->axes[axes_index].value < ctrl_data->axes[axes_index].meta->size_min) {
            duke_out.axis8[i] = ctrl_data->axes[axes_index].meta->size_min;
        }
        else {     
            duke_out.axis8[i] = ctrl_data->axes[axes_index].value;
        }
    }
        memcpy(wired_data->output, (void *)&duke_out, sizeof(duke_out));
    ogx360_process(wired_data->index);
    
}

void ogx360_from_generic(int32_t dev_mode, struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {
    switch (dev_mode) {
        case DEV_KB:
        case DEV_MOUSE:
        //...Future implementations for keyboard and mouse - not ready yet.
        case DEV_PAD:
        default:
            ogx360_ctrl_from_generic(ctrl_data, wired_data);
            break;
    }
}


void ogx360_fb_to_generic(int32_t dev_mode, struct raw_fb *raw_fb_data, struct generic_fb *fb_data) {

    fb_data->wired_id = raw_fb_data->header.wired_id;
    fb_data->type = raw_fb_data->header.type;
    
    // OGX360 HID Output Report format:
    // byte 0: 0x00
    // byte 1: 0x06 (size)
    // bytes 2-3: Left actuator strength (16-bit)
    // bytes 4-5: Right actuator strength (16-bit)
    if (raw_fb_data->header.data_len == 0) {
        fb_data->state = 0;
        fb_data->lf_pwr = 0;
        fb_data->hf_pwr = 0;
    } else {
        uint16_t left_strength = (raw_fb_data->data[3] << 8) | raw_fb_data->data[2];
        uint16_t right_strength = (raw_fb_data->data[5] << 8) | raw_fb_data->data[4];
        fb_data->state = (left_strength || right_strength) ? 1 : 0;
        fb_data->lf_pwr = left_strength;
        fb_data->hf_pwr = right_strength;
    }
}