#include <esp32/rom/ets_sys.h>
#include "adapter/adapter.h"
#include "driver/i2c.h"
#include <soc/i2c_periph.h>
#include "ogx360_i2c.h"
#include <string.h>
#include "adapter/wireless/wireless.h"
#include "bluetooth/host.h"

struct rumble_t
{
    uint32_t wiredId;
    bool success;
    uint32_t type;
};

struct rumble_t rumble_player[4];
bool playerConnected[4] = {0};
bool initialized = false;
esp_err_t resultW;
esp_err_t resultR;


void ogx360_check_connected_controllers()
{
    //ets_printf("OGX360_I2C: Looking for attached OGX360 modules\n");

    for (int i = 0; i < 4; i++)
    {
        const char ping[] = { 0xAA };
        resultW = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 150);
        playerConnected[i] = (resultW == ESP_OK);
        //ets_printf("OGX360_I2C: Player %d %s\n", i, playerConnected[i] ? "Found" : "Not Found");
    }
}

void ogx360_initialize_i2c(void)
{
    i2c_config_t conf = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 21,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0,
    };
    
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    I2C0.timeout.tout = 1048575;
    i2c_set_timeout(I2C_NUM_0, 1048575);

    //ets_printf("OGX360_I2C: Init I2C\n");
}

void ogx360_taskRumble(void *pvParameters)
{
  while(true)
  {
    for (int i = 0; i < 4; i++)
    {
        uint8_t reader[6];
        
        if (rumble_player[i].success && (rumble_player[i].type == BT_XBOX || rumble_player[i].type == BT_PS))
        {
            resultR = i2c_master_read_from_device(I2C_NUM_0, i + 1, (void*)reader, 6, 1048575);

            if (resultR == ESP_OK && reader[1] == 0x06)
                ogx360_acc_toggle_fb(rumble_player[i].wiredId, reader[3], reader[5]);
            else
                ogx360_acc_toggle_fb(rumble_player[i].wiredId, 0, 0);

            rumble_player[i].success = false;
        }
        else if (!rumble_player[i].success && (rumble_player[i].type == BT_XBOX || rumble_player[i].type == BT_PS))
        {
            ogx360_acc_toggle_fb(rumble_player[i].wiredId, 0, 0);
        }
    }
  }
}

void ogx360_acc_toggle_fb(uint32_t wired_id, uint8_t left_motor, uint8_t right_motor)
{
    struct bt_dev *device = NULL;
    struct bt_data *bt_data = NULL;

    bt_host_get_dev_from_out_idx(wired_id, &device);

    if (device) {
        bt_data = &bt_adapter.data[device->ids.id];
        if (bt_data) {
            struct generic_fb fb_data = {0};

            fb_data.wired_id = wired_id;
            fb_data.type = FB_TYPE_RUMBLE;
            fb_data.cycles = 0;
            fb_data.start = 0;
            fb_data.state =(left_motor || right_motor);
            fb_data.left_motor = left_motor;
            fb_data.right_motor = right_motor;
            wireless_fb_from_generic(&fb_data, bt_data);
            bt_hid_feedback(device, bt_data->output);
        }
    }
}

void ogx360_process(uint8_t player, int32_t type)
{
    uint8_t writer[21];
    
    if (!initialized)
    {
        ogx360_initialize_i2c();
        ogx360_check_connected_controllers();
        initialized = true;
        xTaskCreatePinnedToCore(ogx360_taskRumble, "ogx360_taskRumble", 4096, NULL, 1, NULL, 0);
    }

    if (playerConnected[player])
    {        
        memcpy(&writer, &wired_adapter.data[player].output, 21);
        resultW = i2c_master_write_to_device(I2C_NUM_0, player + 1, (void*)writer, 21, 1);

        rumble_player[player].type = type;
        rumble_player[player].wiredId = player;

        if (!rumble_player[player].success)
            rumble_player[player].success = (resultW == ESP_OK);
    }
}