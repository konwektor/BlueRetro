#include "adapter/wired/ogx360.h"
#include <esp32/rom/ets_sys.h>
#include "adapter/adapter.h"
#include "driver/i2c.h"
#include <soc/i2c_periph.h>
#include "ogx360_i2c.h"
#include <string.h>
#include "driver/gpio.h"
#include "tools/util.h"
#include "system/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <sys/time.h>
#include "adapter/wireless/wireless.h"
#include "bluetooth/host.h"
#include "esp_system.h"

//static const char* TAG = "OGX360_I2C";
#define OGX360_I2C_PORT_MAX 4
#define I2C_NUM I2C_NUM_0

bool playerConnected[OGX360_I2C_PORT_MAX] = {false};
//bool previouslyActive[OGX360_I2C_PORT_MAX] = {false};
bool initialized = false;
//static bool i2c_initialized = false;
//static bool config_updated = false;
//static uint16_t current_mask = 0;
//uint32_t gpio_pin[4] = {19, 5, 26, 27};
//int64_t start_time_functionX; //


struct rumble_t
{
uint32_t wiredId;
bool success;
uint32_t type;
};

struct rumble_t rumble_player[4];
//bool playerConnected[4] = {0};
//bool initialized = false;
esp_err_t resultW;
esp_err_t resultR;

uint16_t combine_bytes(uint8_t high_byte, uint8_t low_byte) {
    return (high_byte << 8) | low_byte;
}

esp_err_t i2c_get_period(i2c_port_t i2c_num, int *high_period, int *low_period);
esp_err_t i2c_get_start_timing(i2c_port_t i2c_num, int *setup_time, int *hold_time);
esp_err_t i2c_get_stop_timing(i2c_port_t i2c_num, int *setup_time, int *hold_time);
esp_err_t i2c_get_data_timing(i2c_port_t i2c_num, int *sample_time, int *hold_time);
esp_err_t i2c_get_timeout(i2c_port_t i2c_num, int *timeout);
esp_err_t i2c_get_data_mode(i2c_port_t i2c_num, i2c_trans_mode_t *tx_trans_mode, i2c_trans_mode_t *rx_trans_mode);

// Funkcja do wyswietlania parametrow I2C
void print_i2c_settings(void) {
    i2c_port_t i2c_num = I2C_NUM_0; // Port I2C, ktory chcesz skonfigurowac
    int high_period, low_period;
    int start_setup_time, start_hold_time;
    int stop_setup_time, stop_hold_time;
    int data_sample_time, data_hold_time;
    int timeout;
    i2c_trans_mode_t tx_trans_mode, rx_trans_mode;
    
    // Odczytaj ustawienia I2C
    esp_err_t err;

    // Odczytaj okres SCL
    err = i2c_get_period(i2c_num, &high_period, &low_period);
    if (err != ESP_OK) {
        ets_printf("Error getting SCL period: %s\n", esp_err_to_name(err));
        return;
    }
    ets_printf("I2C Port %d SCL Period:\n", i2c_num);
    ets_printf("High Period: %d clock cycles\n", high_period);
    ets_printf("Low Period: %d clock cycles\n", low_period);

    // Odczytaj timing startu
    err = i2c_get_start_timing(i2c_num, &start_setup_time, &start_hold_time);
    if (err != ESP_OK) {
        ets_printf("Error getting start timing: %s\n", esp_err_to_name(err));
        return;
    }
    ets_printf("I2C Port %d Start Timing:\n", i2c_num);
    ets_printf("Setup Time: %d clock cycles\n", start_setup_time);
    ets_printf("Hold Time: %d clock cycles\n", start_hold_time);

    // Odczytaj timing stopu
    err = i2c_get_stop_timing(i2c_num, &stop_setup_time, &stop_hold_time);
    if (err != ESP_OK) {
        ets_printf("Error getting stop timing: %s\n", esp_err_to_name(err));
        return;
    }
    ets_printf("I2C Port %d Stop Timing:\n", i2c_num);
    ets_printf("Setup Time: %d clock cycles\n", stop_setup_time);
    ets_printf("Hold Time: %d clock cycles\n", stop_hold_time);

    // Odczytaj timing danych
    err = i2c_get_data_timing(i2c_num, &data_sample_time, &data_hold_time);
    if (err != ESP_OK) {
        ets_printf("Error getting data timing: %s\n", esp_err_to_name(err));
        return;
    }
    ets_printf("I2C Port %d Data Timing:\n", i2c_num);
    ets_printf("Sample Time: %d clock cycles\n", data_sample_time);
    ets_printf("Hold Time: %d clock cycles\n", data_hold_time);

    // Odczytaj timeout
    err = i2c_get_timeout(i2c_num, &timeout);
    if (err != ESP_OK) {
        ets_printf("Error getting timeout: %s\n", esp_err_to_name(err));
        return;
    }
    ets_printf("I2C Port %d Timeout:\n", i2c_num);
    ets_printf("Timeout: %d APB cycles\n", timeout);

    // Odczytaj tryb transmisji danych
    err = i2c_get_data_mode(i2c_num, &tx_trans_mode, &rx_trans_mode);
    if (err != ESP_OK) {
        ets_printf("Error getting data mode: %s\n", esp_err_to_name(err));
        return;
    }
    ets_printf("I2C Port %d Data Transfer Mode:\n", i2c_num);
    ets_printf("TX Mode: %d\n", tx_trans_mode);
    ets_printf("RX Mode: %d\n", rx_trans_mode);
}

// Function to print received I2C data
void print_received_data(uint8_t *reader, size_t size) {
    // Clear the current line and move the cursor back to the start
    ets_printf("\r\x1b[32mReceived I2C data: ");

    // Print the data in hex format
    for (size_t i = 0; i < size; i++) {
        ets_printf("0x%02x ", reader[i]);
    }

    // Reset color back to normal
    ets_printf("\x1b[0m");  // Reset the color to default

    // Ensure that the line is flushed to the terminal
    fflush(stdout);
}

 void ogx360_initialize_i2c(void) {
    ets_printf("Initializing I2C...\n");
    i2c_config_t conf = {0}; // Upewnij sie, ze konfiguracja jest poprawnie wyzerowana
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 22;
    conf.scl_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;

    ets_printf("Before calling i2c_param_config\n");
    esp_err_t result = i2c_param_config(I2C_NUM_0, &conf);
    ets_printf("After calling i2c_param_config, result: %d\n", result);

    if (result != ESP_OK) {
        ets_printf("i2c_param_config failed: %d\n", result);
        return;
    }

    result = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (result != ESP_OK) {
        ets_printf("i2c_driver_install failed: %d\n", result);
        return;
    }

    // Sprawdzenie wskaznika i2c
    if (I2C0.timeout.tout != 0) {
        i2c_set_timeout(I2C_NUM_0, 1048575);
    }

    ets_printf("I2C initialization complete.\n");
}





void ogx360_pingSlaves() { 
    // Pinging from ogx360_process only once at first init, then from ogx360_i2c_port_cfg after bitmask update / port switching func
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        const char ping[] = { 0xAA };
        const char disconnectPacket[] = { 0xF0 };

        // If port is not available, attempt to detach the Arduino.
        if (!playerConnected[i]) {
            ets_printf("OGX360_pingSlaves: Detaching Arduino USB on port %d because port is not available\n", i);
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 150);

            if (result != ESP_OK) {
                ets_printf("\x1b[31mOGX360_pingSlaves: Detaching Arduino USB on port %d failed with error code: %d\x1b[0m\n", i, result);  // Print error in red
            } else {
                ets_printf("OGX360_pingSlaves: Arduino USB on port %d detached successfully\n", i);
                playerConnected[i] = false;  // Mark player as disconnected after successful detachment
            }
        } 
        // If the player is connected, attempt to ping the Arduino.
        else {
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 150);
            playerConnected[i] = (result == ESP_OK);  // Update connection status based on ping result
            if (result == ESP_OK) {
                ets_printf("OGX360_pingSlaves: Pinging Arduino on port %d successful, player %d is connected\n", i, i);
            } else {
                ets_printf("\x1b[31mOGX360_pingSlaves: Pinging Arduino on port %d failed with error code: %d\x1b[0m\n", i, result);  // Print error in red
                playerConnected[i] = false;  // Mark player as disconnected if ping fails
            }
        }
    }
}

void ogx360_process(uint8_t player)
{
    uint8_t writer[21];
    //uint8_t reader[6];
    
    if (!initialized)
    {
        // Initialize I2C
        ogx360_initialize_i2c();
		ogx360_pingSlaves();
      /*  for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
            
            if (playerConnected[i]) {
                ets_printf("wywolanie ping slaves wewnatrz ogx360_process\n");
                ogx360_pingSlaves(i);
            }
        }*/
  
        initialized = true;
        }
        //ogx360_pingSlaves(player);
        if (playerConnected[player])
        {
            // Prepare data for writing
            memcpy(&writer, &wired_adapter.data[player].output, 21);
            // Allocate memory for reading response
            uint8_t reader[6];
            
            // Perform combined write and read operation using reapeated start -  no moore misfired reads/writes
            resultW = i2c_master_write_read_device(
                I2C_NUM_0,            // I2C Port number
                player + 1,           // I2C device address
                (void*)writer,        // Data to be written
                21,                   // Length of data to write
                (void*)reader,        // Buffer to store read data
                6,                    // Length of data to read
                10                    // Timeout in ticks
            );
        
        // Handle the rumble_player state based on the result of write/read operation
        rumble_player[player].wiredId = player;
        // Print the values of reader[2], reader[3], reader[4], and reader[5]
        //ets_printf("Reader values: reader[2] = 0x%02x, reader[3] = 0x%02x, reader[4] = 0x%02x, reader[5] = 0x%02x\n", reader[2], reader[3], reader[4], reader[5]);
        // Check if the combined operation was successful
        if (resultW == ESP_OK && reader[1] == 0x06) {
            // Successfully read the response; now extract motor data
            uint16_t left_motor = combine_bytes(reader[2], reader[3]);  // Combine bytes for left motor
            uint16_t right_motor = combine_bytes(reader[4], reader[5]); // Combine bytes for right motor
            if (!rumble_pending) { //rumble blocker if pending rumble activated in by ogx360_ctr_special_action
                // Only apply the read values if there's no pending rumble
                // Toggle force feedback with motor values
               // ets_printf ("sending rumble on");
                ogx360_acc_toggle_fb(rumble_player[player].wiredId, left_motor, right_motor);
            }
            // Mark the operation as successful
            rumble_player[player].success = true;
            } 
            else {
                if (!rumble_pending) {
                    // If the read wasn't successful or the data wasn't valid, disable feedback
                    //ets_printf ("sending rumble oFF");
                    ogx360_acc_toggle_fb(rumble_player[player].wiredId, 0, 0);
                    if (resultW != ESP_OK) {
                       ets_printf ("\x1b[31mI2C Write/Read failed for player %d. Error code: %d\x1b[0m\n", player, resultW);  // Print error in red
                        } else {
                            // ets_printf("\x1b[31mI2C Read failed for player %d. Invalid data received: reader[1] = 0x%02x\x1b[0m\n", player, reader[1]);  // Print error in red
                        }
                        // Mark the operation as unsuccessful
                        rumble_player[player].success = false;
                }
            }
        }
}




       /* resultW = i2c_master_write_to_device(I2C_NUM_0, player + 1, (void*)writer, 21, 1);

        //rumble_player[player].type = type;
        rumble_player[player].wiredId = player;

         if (!rumble_player[player].success)
            rumble_player[player].success = (resultW == ESP_OK);

        // Add reading part here       
        // uint8_t reader[6];
         
        
        if (rumble_player[player].success )
        {
            resultR = i2c_master_read_from_device(I2C_NUM_0, player + 1, (void*)reader, 6, 10);
            

            if (resultR == ESP_OK && reader[1] == 0x06) {
            uint16_t left_motor = combine_bytes(reader[2], reader[3]); // left_motor to dataArray[2:3]
            uint16_t right_motor = combine_bytes(reader[4], reader[5]); // right_motor to dataArray[4:5]
                ogx360_acc_toggle_fb(rumble_player[player].wiredId, left_motor, right_motor);
            }
            else
                ogx360_acc_toggle_fb(rumble_player[player].wiredId, 0, 0);

            rumble_player[player].success = false;
        }
        else if (!rumble_player[player].success )
        {
            ogx360_acc_toggle_fb(rumble_player[player].wiredId, 0, 0);
        }
    
  }
*/
void ogx360_acc_toggle_fb(uint32_t wired_id, uint16_t left_motor, uint16_t right_motor)
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
            fb_data.left_motor = left_motor;// << 16 ;
            fb_data.right_motor = right_motor;// << 16;
            wireless_fb_from_generic(&fb_data, bt_data);
            bt_hid_feedback(device, bt_data->base.output);
        }
    }
}


void ogx360_i2c_init(uint32_t package) {
	ets_printf("pusty init \n");
   // ESP_LOGI(TAG, "Initializing OGX360 I2C");
    ets_printf("zaczyna gpiosy \n");
    // Configure GPIO pins for I2C
    gpio_config_t io_conf = {0};
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pin_bit_mask = (1ULL<<21) | (1ULL<<22);
            gpio_config_iram(&io_conf);
    
    ets_printf("gpio_config_iram(&io_conf) poszedl \n");
    ogx360_i2c_port_cfg(0x0);
    ets_printf("PORT ZERO-ZERO POSZEDL \n");
   
}



/*
void ogx360_i2c_port_cfg(uint16_t mask) {
    ets_printf("Updating mask - port_cfg (mask: 0x%04X)\n", mask);
    
    for (int i = 0; i < 4; i++) {
        bool portActive = (mask & (1 << i)) != 0;
        ets_printf("OGX360_I2C_PORT_CFG: PORT %d %s\n", i, portActive ? "ACTIVE" : "INACTIVE");

        // Handle new connection when port is active, not initialized, and no player is connected.
        if (portActive && !initialized && !playerConnected[i]) {
            playerConnected[i] = true;
            ets_printf("OGX360_I2C_PORT_CFG: Player %d connected on port %d\n", i, i);

        }

        // Handle reconnect when port is active, initialized, but player is not connected.
        else {
            if (portActive && initialized && !playerConnected[i]) {
                const char ping[] = { 0xAA };
                esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 150);
            
            if (result == ESP_OK) {
                ets_printf("OGX360_I2C_PORT_CFG: Pinging Arduino on port %d OK\n", i);
                const char attachUsb[] = { 0xF1 };
                result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)attachUsb, sizeof(attachUsb), 150);
                if (result == ESP_OK) {
                    ets_printf("OGX360_I2C_PORT_CFG: Attaching Arduino's USB to port %d OK\n", i);
                    playerConnected[i] = true;
                    
                } else {
                    ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Attaching Arduino's USB to port %d failed with error code: %d\x1b[0m\n", i, result);
                    playerConnected[i] = false;  // Mark player as disconnected if ping fails
                    ets_printf("OGX360_I2C_PORT_CFG: Attaching Arduino's USB to port %d OK\n", i);
                }
            } else {
                ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Pinging Arduino on port %d failed with error code: %d\x1b[0m\n", i, result);

            }
        }

        // Handle disconnection when port goes inactive but player was previously connected.
        if (!portActive && initialized && playerConnected[i]) {
            
            const char disconnectPacket[] = { 0xF0 };
            ets_printf("OGX360_I2C_PORT_CFG: Because port %d went inactive, detaching Arduino USB\n", i);
            
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 150);
            
            if (result == ESP_OK) {
                ets_printf("OGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d OK\n", i);
                playerConnected[i] = false;
                
            } else {
                ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d failed. Error code: %d\x1b[0m\n", i, result);
            }
        }
    }
*/
void ogx360_i2c_port_cfg(uint16_t mask) {
    ets_printf("Updating mask - port_cfg (mask: 0x%04X)\n", mask);
    
    for (int i = 0; i < 4; i++) {
        bool portActive = (mask & (1 << i)) != 0;
        ets_printf("OGX360_I2C_PORT_CFG: PORT %d %s\n", i, portActive ? "ACTIVE" : "INACTIVE");

        // Block 1: If this block runs, no other blocks should be executed.
        if (portActive && !initialized && !playerConnected[i]) {
            playerConnected[i] = true;  // Mark player as connected
            ets_printf("OGX360_I2C_PORT_CFG: Player %d connected on port %d\n", i, i);
        } 
        // If Block 1 is not executed, execute both Block 2 and Block 3.
        else {
            // Block 2: Handle reconnection when port is active, initialized, but player is not connected.
            if (portActive && initialized && !playerConnected[i]) {
                const char ping[] = { 0xAA };
                esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 150);
                
                if (result == ESP_OK) {
                    ets_printf("OGX360_I2C_PORT_CFG: Pinging Arduino on port %d OK\n", i);
                    
                    const char attachUsb[] = { 0xF1 };
                    result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)attachUsb, sizeof(attachUsb), 150);
                    
                    if (result == ESP_OK) {
                        ets_printf("OGX360_I2C_PORT_CFG: Attaching Arduino's USB to port %d OK\n", i);
                        playerConnected[i] = true;  // Mark player as connected
                    } else {
                        ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Attaching Arduino's USB to port %d failed with error code: %d\x1b[0m\n", i, result);
                    }
                } else {
                    ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Pinging Arduino on port %d failed with error code: %d\x1b[0m\n", i, result);
                }
            }

            // Block 3: Handle disconnection when port is inactive but player was previously connected.
            if (!portActive && initialized && playerConnected[i]) {
                const char disconnectPacket[] = { 0xF0 };
                ets_printf("OGX360_I2C_PORT_CFG: Because port %d went inactive, detaching Arduino USB\n", i);
                
                esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 150);
                
                if (result == ESP_OK) {
                    ets_printf("OGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d OK\n", i);
                    playerConnected[i] = false;  // Mark player as disconnected
                } else {
                    ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d failed. Error code: %d\x1b[0m\n", i, result);
                }
            }
        }
    }
}
