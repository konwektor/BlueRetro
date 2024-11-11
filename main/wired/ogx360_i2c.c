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


#define OGX360_I2C_PORT_MAX 4
#define I2C_NUM I2C_NUM_0

bool portActive[OGX360_I2C_PORT_MAX] = {false}; 
bool playerConnected[OGX360_I2C_PORT_MAX] = {false};
//static bool rumble_off_state[4] = {false};
bool initialized = false;
bool i2c_done = false;

struct rumble_t
{
uint32_t wiredId;
//bool success;
//uint32_t type; //could be usefull in future, actions depending on btgamepad type 
};
struct rumble_t rumble_player[4];

uint16_t combine_bytes(uint8_t high_byte, uint8_t low_byte) {
    return (high_byte << 8) | low_byte;
}

void ogx360_initialize_i2c(void) {
    if  (i2c_done !=0) { //If wired_adapter_reinit was done, dont initialize  already initialized i2c!
        return;
    } else {
        ets_printf("OGX360_INITIALIZE_I2C - setting I2C config\n");
        i2c_config_t conf = {0}; // always do 0 cfg
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = 22;
        conf.scl_io_num = 21;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 400000;
        conf.clk_flags = 0;
        esp_err_t result = i2c_param_config(I2C_NUM_0, &conf);
        if (result != ESP_OK) {
            ets_printf("OGX360_INITIALIZE_I2C - i2c_param_config failed: %d\n", result);
            return;
        }
        result = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
        if (result != ESP_OK) {
            ets_printf("OGX360_INITIALIZE_I2C - i2c_driver_install failed: %d\n", result);
            return;
        }
       
        if (I2C0.timeout.tout != 0) {
            i2c_set_timeout(I2C_NUM_0, 1048575);
        }
        ets_printf("OGX360_INITIALIZE_I2C - I2C initialization complete.\n");
        i2c_done = true;
    }
}

void ogx360_pingSlaves() { 
    // Only ping if I2C is initialized
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        // Skip if port state matches connection state
        if (portActive[i] == playerConnected[i]) {
            continue;
        }

        if (portActive[i]) {
            // Port is active but player not connected - try to connect
            const char ping[] = { 0xAA };
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 150);
            playerConnected[i] = (result == ESP_OK);
            ets_printf("OGX360_pingSlaves: Port %d ping %s\n", i, (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");
            if (result != ESP_OK) {
                portActive[i] = false;
            }
        } else {
            // Port is inactive but player still connected - disconnect
            const char disconnectPacket[] = { 0xF0 };
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 150);
            
            if (result == ESP_OK) {
                playerConnected[i] = false;
                ets_printf("OGX360_pingSlaves: Port %d disconnected successfully\n", i);
            } else {
                ets_printf("\x1b[31mOGX360_pingSlaves: Port %d disconnect failed\x1b[0m: %d\n", i, result);
                //After detach failure we need to disconnect arduino from xbox USB port! also TODO - implement arduino hard reset over gpio pin? 
                //Or wired reinit
            }
        }
    }
    
}



void ogx360_process(uint8_t player) {
    uint8_t reader[6]= {0};
    int len = sizeof(reader);
    if (!initialized) {
        ogx360_initialize_i2c();       
        ets_printf("OGX360_PROCESS: i2c initialized\n");
        memset(playerConnected, 0, sizeof(playerConnected));
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        ogx360_pingSlaves();
        initialized = true;        
        //return;
    }
    
    if (portActive[player] != playerConnected[player]) {

        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        ogx360_pingSlaves();
    }

    if (playerConnected[player]) {
        // Perform combined write and read operation using reapeated start - no more misfired reads/writes
        esp_err_t result = i2c_master_write_read_device (
        I2C_NUM_0,                                     // I2C Port number
        player + 1,                                    // I2C device address
        (void*)&wired_adapter.data[player].output,     // Data to be written
        21,                                            // Length of data to write
        (void*)reader,                                 // Buffer to store read data
        6,                                             // Length of data to read
        10                                             // Timeout in ticks
        );

        if (result == ESP_OK) {

            if (reader[0] == 0x00) {

                if (reader[1] == 0x06) {
                                        
                    if (len ==0x06 ) {
                        
                        // Check data after I2C read
                        struct raw_fb fb_data = {0};
                        fb_data.header.wired_id = player;
                        fb_data.header.type = FB_TYPE_RUMBLE;
                        fb_data.header.data_len = 6;
                        
                        //if (result == ESP_OK && reader[0] == 0x00 && reader[1] == 0x06) {
                        // Valid data, copy rumble information to fb_data for ogx360_fb_to_generic function
                        fb_data.data[0] = reader[0];
                        fb_data.data[1] = reader[1];
                        fb_data.data[2] = reader[2];  // Left motor low byte
                        fb_data.data[3] = reader[3];  // Left motor high byte
                        fb_data.data[4] = reader[4];  // Right motor low byte
                        fb_data.data[5] = reader[5];  // Right motor high byte
                    
                        // Queue the feedback data for further processing
                        adapter_q_fb(&fb_data);
                        if (result != ESP_OK) {
                            ets_printf ("\x1b[31mI2C Write/Read failed for player %d. Error code: %d\x1b[0m\n", player, result);  // Print error in red
                        }else {
                            ets_printf("\x1b[31mI2C Read failed for player %d. Invalid data received: reader[1] = 0x%02x\x1b[0m\n", player, reader[1]);
                            // Invalid data; send zeros to stop rumble if needed
                            fb_data.header.data_len = 0;  // Indicate invalid data to stop any rumble
                        }
                    // Slave nie ma nowych danych
                    ESP_LOGI("I2C", "No new data from slave");
                    return; // Zakończ operację
                    }
                }
            }
        }      
    }
}    
        

void ogx360_i2c_init(uint32_t package) {
	ets_printf("Starting dummy ogx360_i2c_init  \n");
    //Set all ogx360 ports disabled
    initialized = false;
    ogx360_i2c_port_cfg(0x0);
    ets_printf("Dummy ogx360_i2c_init done \n");
    //TODO - 
   
}



void ogx360_i2c_port_cfg(uint16_t mask) {
    ets_printf("OGX360_I2C_PORT_CFG - Updating port_cfg mask: 0x%04X\n", mask);
    
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        bool was_active = portActive[i];
        portActive[i] = (mask & (1 << i)) != 0;
        ets_printf("OGX360_I2C_PORT_CFG: Port %d %s\n", i, portActive[i] ? "ACTIVE" : "INACTIVE");
       
        //Perform I2C operations only after initialization
        if (initialized && portActive[i] != was_active) {
            if (!portActive[i] && playerConnected[i]) {
                const char disconnectPacket[] = { 0xF0 };
                ets_printf("OGX360_I2C_PORT_CFG: Port %d inactive, detaching Arduino USB\n", i);
                esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 15);
                
                if (result == ESP_OK) {
                    ets_printf("OGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d OK\n", i);
                } else {
                    //print error in red
                    ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d failed. Error code: %d\x1b[0m\n", i, result);
                }
                playerConnected[i] = false;
            }
            // Let ogx360_pingSlaves handle connections after initialization
        }
       
    }
}