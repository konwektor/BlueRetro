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
static bool rumble_off_state[4] = {false};
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
    if  (i2c_done !=0) {
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
        // Sprawdzenie wskaznika i2c
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
            if (result == !ESP_OK) {
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
                // we need disconnect arduino from xbox USB port , also after detach failure - implement arduino hard reset over gpio pin? 

            }
        }
    }

    //  final connection states
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        ets_printf("Player %d is %sconnected\n", i, playerConnected[i] ? "" : "not ");
       // if (playerConnected[i]) {
         //   vTaskDelay(150 / portTICK_PERIOD_MS);
           // trigger_special_rumble(i, i+1);
        //}
    }
}

/*        // If port is not available, attempt to detach the Arduino.
        if (!playerConnected[i]) {
            ets_printf("OGX360_pingSlaves: Skipp ping Arduino on port %d because port is not available\n", i);

            return;
        }
        if (!playerConnected[i]) {
            ets_printf("OGX360_pingSlaves: Detaching Arduino USB on port %d because port is not available\n", i);
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 150);
            playerConnected[i] = (result == ESP_OK);
            if (result != ESP_OK) {
                ets_printf("\x1b[31mOGX360_pingSlaves: Detaching Arduino USB on port %d failed with error code: %d\x1b[0m\n", i, result);  // Print error in red
            } else {
                ets_printf("OGX360_pingSlaves: Arduino USB on port %d detached successfully\n", i);
                playerConnected[i] = false;  // Mark player as disconnected after successful detachment
            }
        }

        // If the player is connected, attempt to ping the Arduino.
        //else {
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 150);
            playerConnected[i] = (result == ESP_OK);  // Update connection status based on ping result
            if (result == ESP_OK) {
                ets_printf("OGX360_pingSlaves: Pinging Arduino on port %d successful, player %d is connected\n", i, i);
            } else {
                ets_printf("\x1b[31mOGX360_pingSlaves: Pinging Arduino on port %d failed with error code: %d\x1b[0m\n", i, result);  // Print error in red
            }
        //}
    }
}*/

void ogx360_process(uint8_t player)
{
    if (!initialized) {
        ogx360_initialize_i2c();       
        ets_printf("OGX360_PROCESS: i2c initialized\n");
        ogx360_pingSlaves();
        initialized = true;        
        return;
    }
    
    if (portActive[player] != playerConnected[player]) {
        ogx360_pingSlaves();
    }

    if (playerConnected[player]) {
        uint8_t reader[6];
        // Perform combined write and read operation using reapeated start -  no more misfired reads/writes
        esp_err_t result = i2c_master_write_read_device(
            I2C_NUM_0,                                       // I2C Port number
            player + 1,                                      // I2C device address
            (void*)&wired_adapter.data[player].output,       // Data to be written
            21,                                              // Length of data to write
            (void*)reader,                                   // Buffer to store read data
            6,                                               // Length of data to read
            10                                               // Timeout in ticks
        );
        
        rumble_player[player].wiredId = player;

        // Check if the R/W operation was successful
        if (result == ESP_OK && reader[1] == 0x06) {
            // Handle the rumble_player state based on the result of write/read operation
            uint16_t left_motor = combine_bytes(reader[2], reader[3]);          // Combine bytes for left motor
            uint16_t right_motor = combine_bytes(reader[4], reader[5]);         // Combine bytes for right motor
            //if rumble pending true - rumble is activated by ogx360_ctr_special_action and is blocking other rumble calls on present port til timer ends
            if (!rumble_pending[player]) {  // proceed with values only if there's no pending rumble
                
                if (left_motor != 0 || right_motor != 0) {
                    rumble_off_state[player] = false;
                    ogx360_acc_toggle_fb(rumble_player[player].wiredId, 0, left_motor, right_motor);
                
                } else {
                    if (left_motor == 0 && right_motor == 0) {
                        if (rumble_off_state[player]) {
                            return;
                        }
                        rumble_off_state[player] = true;
                        ogx360_acc_toggle_fb(rumble_player[player].wiredId, 0, 0, 0);
                    }
                }
            }
        } else {
            if (!rumble_off_state[player] && !rumble_pending[player]) {
                // If the read wasn't successful or the data wasn't valid, disable feedback
                ets_printf ("sending rumble off");
                ogx360_acc_toggle_fb(rumble_player[player].wiredId, 0, 0, 0);
                rumble_off_state[player] = true;
                if (result != ESP_OK) {
                ets_printf ("\x1b[31mI2C Write/Read failed for player %d. Error code: %d\x1b[0m\n", player, result);  // Print error in red
                //} else {
                    // ets_printf("\x1b[31mI2C Read failed for player %d. Invalid data received: reader[1] = 0x%02x\x1b[0m\n", player, reader[1]);
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
    //}
    ets_printf("Dummy ogx360_i2c_init done \n"); 
   
}



void ogx360_i2c_port_cfg(uint16_t mask) {
    ets_printf("OGX360_I2C_PORT_CFG - Updating port_cfg mask: 0x%04X\n", mask);
    
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        bool was_active = portActive[i];
        portActive[i] = (mask & (1 << i)) != 0;
        ets_printf("OGX360_I2C_PORT_CFG: Port %d %s\n", i, portActive[i] ? "ACTIVE" : "INACTIVE");

        // Before I2C initialization, only update flags
        if (!initialized) {
           // if (playerConnected[i]) {
                ets_printf("OGX360_I2C_PORT_CFG: i2C not initialized, setting playerConnected[%d] false on port %d\n", i, i);
                playerConnected[i] = false;
           // }
            continue;
        }

        // Only perform I2C operations after initialization
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
        if (initialized) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            trigger_special_rumble(i, i+1);
        }
    }
}

