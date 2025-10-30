
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
#include "hal/gpio_ll.h"
#include <hal/clk_gate_ll.h>
#include "hal/i2c_ll.h"
#include "esp_private/periph_ctrl.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "esp_attr.h"
#include "adapter/wired/ogx360.h"


#define OGX360_I2C_PORT_MAX 4
#define sda_pin 22
#define scl_pin 21
  #define I2C_NUM I2C_NUM_0
//#define APB_CLK_HZ (80 * 1000 * 1000)  // Można zmienić globalnie



static bool portActive[OGX360_I2C_PORT_MAX] = {false}; 
static bool playerConnected[OGX360_I2C_PORT_MAX] = {false};
static bool rumble_off_state[4] = {false};
static bool initialized = false;
static bool ports_update_pending = false;


// Liczniki wywołań
//static uint32_t process_call_count[OGX360_I2C_PORT_MAX] = {0};
//static uint32_t i2c_rw_call_count[OGX360_I2C_PORT_MAX] = {0};
//static uint64_t last_reset_time = 0;
//void ogx360_polling_task(void);

const char ping[] = { 0xAA };
const char disconnectPacket[] = { 0xF0 };



 // Funkcja niskopoziomowego zapisu I2C
static inline esp_err_t i2c_ll_master_write(i2c_port_t i2c_num, uint8_t slave_addr, const uint8_t *data, size_t data_len) {
    i2c_dev_t *hw = I2C_LL_GET_HW(i2c_num);

    // Clear TX FIFO & interrupts
    i2c_ll_txfifo_rst(hw);
    hw->int_clr.val = 0xFFFFFFFF;

    // Address byte + data
    uint8_t buf[1 + data_len];
    buf[0] = (slave_addr << 1) | 0;
    memcpy(&buf[1], data, data_len);

    // Fill FIFO in safe chunks
    size_t sent = 0;
    while (sent < 1 + data_len) {
        size_t chunk = MIN(16, (1 + data_len) - sent);
        i2c_ll_write_txfifo(hw, buf + sent, chunk);
        sent += chunk;
    }

    // Prepare commands
    i2c_ll_hw_cmd_t cmd = {0};
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);

    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1 + data_len;
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1);

    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 2);

    // Rozpocznij transmisję
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    
    i2c_ll_start_trans(hw);

    // Start transfer
   
    int timeout = 2000;
    while (!i2c_ll_master_is_cmd_done(hw, 2) && timeout--) {
        ets_delay_us(10);
    }

    if (hw->int_raw.ack_err) {
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        i2c_ll_master_fsm_rst(hw);
        hw->int_clr.ack_err = 1;
        return ESP_FAIL;
    }

    return (timeout > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void ogx360_i2c_ping1(uint8_t player) {
    // ogx360_initialize_i2c();
    //esp_err_t result = i2c_ll_master_write(I2C_NUM_0, 1, (void*)ping, sizeof(ping));
    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, player + 1, (void*)ping, sizeof(ping), 10);
    playerConnected[player] = (result == ESP_OK);
     ets_printf("ogx360_i2c_ping1: Player %d ping %s\n",player +1, (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");
    //  initialized =true;
}

void ogx360_i2c_disconnectPacket1(uint8_t player) {
     esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, player + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 10);
     playerConnected[player] = (result == ESP_OK);
     ets_printf("ogx360_i2c_disconnectPacket1: player %d disconnectPacket %s\n",player + 1, (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");
}


void ogx360_i2c_ping_ll(void) {
     i2c_dev_t *hw = I2C_LL_GET_HW(I2C_NUM_0);
     // --- Przygotowanie danych do wysłania ---
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll --- Ustawienie komend: RESTART, WRITE, STOP ---  \n");
    
    uint8_t buf[2];
    buf[0] = (0x01 << 1) | 0;  // adres 0x01 + bit W=0
    buf[1] = 0xAA;            // wysyłany bajt
    i2c_ll_write_txfifo(hw, buf, 2);

    // --- Ustawienie komend: RESTART, WRITE, WRITE, STOP ---
    i2c_ll_hw_cmd_t cmd = {0};
    cmd.op_code = I2C_LL_CMD_RESTART;
    cmd.byte_num = 0;
    cmd.ack_en = 0;  // oczekujemy 
    cmd.ack_exp = 0; // oczekiwany 
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll : RESTART POSZEDL  \n");
    
    //WRITE adress
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1);  // komenda 1: write adres
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll : WRITE ADRESS POSZEDL \n"); 

    //WRITE data
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;   
    i2c_ll_master_write_cmd_reg(hw, cmd, 2);  // komenda 2: write dane
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll :  WRITE 1  BAJT POSZEDL \n");

    // STOP
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.byte_num = 0;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 3);  // komenda 3: stop
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll :  KOMENDA STOP POSZLA \n");

    // --- Clear error flags before start ---
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    
    // --- Rozpoczęcie transmisji ---
   i2c_ll_start_trans(hw);
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll : poszlo i2c_ll_master_trans_start  \n");
 
    // --- Oczekiwanie na zakończenie transmisji (polecenie STOP) ---
    while (!i2c_ll_master_is_cmd_done(hw, 3)) {    
    // Czekamy...
    // TODO: ADD timeout/RETRY lajter for while !i2c_ll_master_is_cmd_done
    //int retries = 3;
    // while (retries--) {
    // ... [transmission code] ...
    //if (!hw->int_raw.ack_err) break;
    //ets_delay_us(100000); // 100ms delay
    //}
    }
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll  :  poszlo i2c_ll: master_is_cmd_done  \n");

    // sprawdzenie błędów
    // --- Critical Error Check ---
    if (hw->int_raw.ack_err) {
        ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll     I2C ERROR: NACK received!\n");
        // Handle error: reset FIFOs, clear bus, etc.
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        i2c_ll_master_fsm_rst(hw);  // Reset FSM
        hw->int_clr.ack_err = 1;    // Clear error flag
    }
}




void ogx360_i2c_disconnect_ll(void) {
    i2c_dev_t *hw = I2C_LL_GET_HW(I2C_NUM_0);
   
    // --- Przygotowanie danych do wysłania ---
    ets_printf("ogx360_LOW_LEVEL_i2c_disconnect :  --- Ustawienie komend: RESTART, WRITE, STOP ---  \n");
    
    uint8_t buf[2];
    buf[0] = (0x01 << 1) | 0;  // adres 0x01 + bit W=0
    buf[1] = 0xF0;            // wysyłany bajt
    i2c_ll_write_txfifo(hw, buf, 2);

    // --- Ustawienie komend: RESTART, WRITE, WRITE, STOP ---
    i2c_ll_hw_cmd_t cmd = {0};
    cmd.op_code = I2C_LL_CMD_RESTART;
    cmd.byte_num = 0;
    cmd.ack_en = 0;  // oczekujemy 
    cmd.ack_exp = 0; // oczekiwany 
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);
    ets_printf("ogx360_LOW_LEVEL_i2c_disconnect : RESTART POSZEDL  \n");
    
    //WRITE adress
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1);  // komenda 1: write adres
    ets_printf("ogx360_LOW_LEVEL_i2c_disconnect :  WRITE ADRESS POSZEDL \n"); 

    //WRITE data
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;   
    i2c_ll_master_write_cmd_reg(hw, cmd, 2);  // komenda 2: write dane
    ets_printf("ogx360_LOW_LEVEL_i2c_disconnect :  WRITE 1  BAJT POSZEDL \n");

    // STOP
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.byte_num = 0;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 3);  // komenda 3: stop
    ets_printf("ogx360_LOW_LEVEL_i2c_disconnect :  KOMENDA STOP POSZLA \n");

    // --- Clear error flags before start ---
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    
    // --- Rozpoczęcie transmisji ---
   i2c_ll_start_trans(hw);
    ets_printf("ogx360_LOW_LEVEL_i2c_disconnect : poszlo i2c_ll_master_trans_start  \n");
 
    // --- Oczekiwanie na zakończenie transmisji (polecenie STOP) ---
    while (!i2c_ll_master_is_cmd_done(hw, 3)) {    
    // Czekamy...
    // TODO: ADD timeout/RETRY lajter for while !i2c_ll_master_is_cmd_done
    //int retries = 3;
    // while (retries--) {
    // ... [transmission code] ...
    //if (!hw->int_raw.ack_err) break;
    //ets_delay_us(100000); // 100ms delay
    //}
    }
    ets_printf("ogx360_LOW_LEVEL_i2c_disconnect : master_is_cmd_done  \n");

    // sprawdzenie błędów
    // --- Critical Error Check ---
    if (hw->int_raw.ack_err) {
        ets_printf("ogx360_LOW_LEVEL_i2c_disconnect : I2C ERROR: NACK received!\n");
        // Handle error: reset FIFOs, clear bus, etc.
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        i2c_ll_master_fsm_rst(hw);  // Reset FSM
        hw->int_clr.ack_err = 1;    // Clear error flag
    }
}




void ogx360_initialize_i2c(void) {
    static bool i2c_done = false;

     ets_printf("OGX360_INITIALIZE_I2C - starting\n");    
    if  (i2c_done) { //If wired_adapter_reinit was done, dont initialize  already initialized i2c!
        ets_printf("OGX360_INITIALIZE_I2C - If wired_adapter_reinit was done, dont initialize  already initialized i2c - returning from functionn!\n");
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
           // ets_printf("OGX360_INITIALIZE_I2C - i2c_param_config failed: %d\n", result);
            return;
        }
        result = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
        if (result != ESP_OK) {
            //ets_printf("OGX360_INITIALIZE_I2C - i2c_driver_install failed: %d\n", result);
            return;
        }
        // Sprawdzenie wskaznika i2c
        if (I2C0.timeout.tout != 0) {
            i2c_set_timeout(I2C_NUM_0, 1048575);
        }
        ets_printf("OGX360_INITIALIZE_I2C - I2C initialization complete.\n");
        i2c_done = true;
        initialized =true;
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
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 10);
            playerConnected[i] = (result == ESP_OK);
            ets_printf("OGX360_pingSlaves: Port %d ping %s\n", i, (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");
            if (result != ESP_OK) {
                portActive[i] = false;
            }
        } else {
            // Port is inactive but player still connected - 
            const char disconnectPacket[] = { 0xF0 };//arduinos USB D+,D- pins going to high-Z state/not visible anymore by USB host
            esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 10);
            
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
/*
static inline esp_err_t i2c_ll_master_write_read(i2c_port_t port, uint8_t slave_addr,
                                   const uint8_t *write_data, size_t write_len,
                                   uint8_t *read_data, size_t read_len) {
                                     i2c_dev_t *hw = (port == 0) ? &I2C0 : &I2C1;
    if (write_len > 0) {
        uint8_t write_buf[1 + write_len];
        write_buf[0] = (slave_addr << 1) | 0;  // Write
        memcpy(&write_buf[1], write_data, write_len);
        i2c_ll_write_txfifo(hw, write_buf, 1 + write_len);
    }

    // WRITE phase
    i2c_ll_hw_cmd_t cmd = {0};
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);

    for (int i = 0; i < (1 + write_len); i++) {
        cmd.op_code = I2C_LL_CMD_WRITE;
        cmd.byte_num = 1;
        cmd.ack_en = 1;
        cmd.ack_exp = 0;
        i2c_ll_master_write_cmd_reg(hw, cmd, 1 + i);
    }

    // READ phase
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1 + write_len + 1);

    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    uint8_t addr_read = (slave_addr << 1) | 1;
    i2c_ll_write_txfifo(hw, &addr_read, 1);
    i2c_ll_master_write_cmd_reg(hw, cmd, 1 + write_len + 2);

    // READ bytes
    for (int i = 0; i < read_len; i++) {
        cmd.op_code = I2C_LL_CMD_READ;
        cmd.byte_num = 1;
        cmd.ack_en = 1;
        cmd.ack_val = (i == read_len - 1) ? 1 : 0;  // NACK on last byte
        i2c_ll_master_write_cmd_reg(hw, cmd, 1 + write_len + 3 + i);
    }

    // STOP
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1 + write_len + 3 + read_len);

    // Start
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    
    i2c_ll_master_trans_start(hw);

    int timeout = 2000;
    while (!i2c_ll_master_is_cmd_done(hw, 1 + write_len + 3 + read_len) && timeout--) {
        ets_delay_us(10);
    }

    if (timeout <= 0) return ESP_ERR_TIMEOUT;

    // Read data from FIFO
    i2c_ll_read_rxfifo(hw, read_data, read_len);

    return ESP_OK;
}


    // Funkcja niskopoziomowego zapisu I2C
static inline esp_err_t i2c_ll_master_write(i2c_port_t port, uint8_t slave_addr, const uint8_t *data, size_t data_len) {

    // Get hardware register pointer from port number
     i2c_dev_t *hw = I2C_LL_GET_HW(I2C_NUM_0);
    //i2c_dev_t *hw = (port == 0) ? &I2C0 : &I2C1;

    // Przygotuj bufor: adres + dane
    uint8_t buf[1 + data_len];
    buf[0] = (slave_addr << 1) | 0;  // Adres 7-bit + bit W=0
    for (size_t i = 0; i < data_len; i++) {
        buf[i + 1] = data[i];
    }

    // Zapis do FIFO TX
    i2c_ll_write_txfifo(hw, buf, 1 + data_len);

    // Konfiguracja komend
    i2c_ll_hw_cmd_t cmd = {0};

    // START (komenda 0)
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);

    // Komendy WRITE dla każdego bajtu
    for (int i = 0; i < (1 + data_len); i++) {
        cmd.op_code = I2C_LL_CMD_WRITE;
        cmd.byte_num = 1;
        cmd.ack_en = 1;    // Wymagaj ACK
        cmd.ack_exp = 0;   // Oczekuj ACK (0)
        i2c_ll_master_write_cmd_reg(hw, cmd, 1 + i);
    }

    // STOP (ostatnia komenda)
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1 + data_len + 1);

    // Rozpocznij transmisję
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    
    i2c_ll_master_trans_start(hw);

    // Czekaj na zakończenie z timeoutem
    int timeout = 1000;
    while (!i2c_ll_master_is_cmd_done(hw, 1 + data_len + 1) && timeout--) {
        ets_delay_us(10);
    }
    
    return timeout > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
    if (hw->int_raw.ack_err) {
        ets_printf("I2C ERROR: NACK received!\n");
        // Handle error: reset FIFOs, clear bus, etc.
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        i2c_ll_master_fsm_rst(hw);  // Reset FSM
        hw->int_clr.ack_err = 1;    // Clear error flag
    }
}*/


/*
void ogx360_process(uint8_t player) {
    //uint8_t reader[6] = {0};
    //int len = sizeof(reader);
    ets_printf("OGX360_PROCESS: starting\n");

    if (!initialized) {
       // ogx360_initialize_i2c();       
        ets_printf("OGX360_PROCESS: not initialized - init and return\n");
        memset(playerConnected, 0, sizeof(playerConnected));
       // ogx360_pingSlaves();
        initialized = true;        
        return;
    }

    if (portActive[player] != playerConnected[player]) {  
        ets_printf("OGX360_PROCESS: Port %d ACTIVATED, player not -quit\n", player);
        const uint8_t ping = 0xAA;
        esp_err_t result = i2c_ll_master_write(I2C_NUM_0, player + 1, &ping, 1);
        if (result == ESP_OK) {
            ets_printf("Pinging Arduino on port %d OK\n", player);
        playerConnected[player] = true;
        } else {
            ets_printf("\x1b[31mPinging Arduino failed! Error: %d\x1b[0m\n", result);
           // ets_printf("Setting port %d as false\n", player);
           // portActive[player] = false;
        }
        return;
        //ets_printf("OGX360_PROCESS: ping slaves skipping\n");
        // ogx360_pingSlaves();
    }

    //if (playerConnected[player]) {
        ets_printf("OGX360_PROCESS: starting write   \n");
            esp_err_t result = i2c_ll_master_write(            
            I2C_NUM_0,
            player + 1,    // Slave address
            (uint8_t*)&wired_adapter.data[player].output, // write buffer
            21            // write length
        );

        if (result == ESP_OK) {
            ets_printf("OGX360_PROCESS:write for player  %d OK\n", player);
        } else {
            ets_printf("\x1b[31mOGX360_PROCESS:Write failed! Error: %d\x1b[0m\n", result);
        }
    //}
}
*/
    


    /*if (playerConnected[player]) {

        
         ets_printf("OGX360_PROCESS: starting write/read  \n");
        i2c_dev_t *hw = I2C_LL_GET_HW(I2C_NUM_0);

        esp_err_t result = i2c_ll_master_write_read(
            I2C_NUM_0,
            player + 1,    // Slave address
            (uint8_t*)&wired_adapter.data[player].output, // write buffer
            21,            // write length
            reader,        // read buffer
            6              // read length
        );

        if (result == ESP_OK) {
            struct raw_fb fb_data = {0};
            fb_data.header.wired_id = player;
            fb_data.header.type = FB_TYPE_RUMBLE;
            fb_data.header.data_len = 6;

            //if (result == ESP_OK && reader[0] == 0x00 && reader[1] == 0x06) {
            // Valid data, copy rumble information to fb_data for ogx360_fb_to_generic functio
            fb_data.data[0] = reader[0];
            fb_data.data[1] = reader[1];
            fb_data.data[2] = reader[2];  // Left motor low byte
            fb_data.data[3] = reader[3];  // Left motor high byte
            fb_data.data[4] = reader[4];  // Right motor low byte
            fb_data.data[5] = reader[5];  // Right motor high byte
            // Queue the feedback data for further processing
            adapter_q_fb(&fb_data);
        }
        // optionally handle errors here
    }*/
    // dalsza część kodu...
void ogx360_ll_process(uint8_t player) {
    if (!initialized) {
    ets_printf("OGX360_LL_PROCESS: i2c not initialized\n");
   ogx360_initialize_i2c();       
    }
    esp_err_t result = i2c_ll_master_write(
        I2C_NUM_0,
        player + 1,
        (void*)&wired_adapter.data[player].output,
        21
    );

  /*  esp_err_t result = i2c_master_write_to_device(
        I2C_NUM_0,
        player + 1,
        (void*)&wired_adapter.data[player].output,
        21,
        10
    );
    */
    ets_printf("ogx360_LL_i2c_write:  %s\n", (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");

}

void ogx360_process(uint8_t player) {
    //no i2c operations during ports update -else timeouts coz of pinging devices
    if (ports_update_pending) {
        ets_printf("OGX360_PROCESS:ports are updating -  quiting\n");        

        return;
    }
    

    // Inicjalizacja timera przy pierwszym wywołaniu
   // if (last_reset_time == 0) {
   //     last_reset_time = esp_timer_get_time();
   // }
    // Zwiększ licznik wywołań dla danego gracza
   // process_call_count[player]++;

   //ets_printf("OGX360_PROCESS:starting\n");
   // Allocate memory for reading response
   uint8_t reader[6]= {0};
   int len = sizeof(reader);
   
   if (!initialized) {
    ets_printf("OGX360_PROCESS: i2c not initialized\n");
  ogx360_initialize_i2c();       
  //  memset(playerConnected, 0, sizeof(playerConnected));
   // vTaskDelay(100 / portTICK_PERIOD_MS);
    //ogx360_pingSlaves();
 // initialized = true;
   // ets_printf("OGX360_PROCESS: i2c initialized\n");        
    return;
    }
    
  /*  if (portActive[player] != playerConnected[player]) {

        ets_printf("OGX360_PROCESS: player not conected, goin to ping slave\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ogx360_pingSlaves();
        ets_printf("OGX360_PROCESS: ping slaves done w8 before quit\n");        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ets_printf("OGX360_PROCESS:vTaskDelay expired, quiting\n");        
        return;
      
    }
        */
    
    //ets_printf("OGX360_PROCESS: if player connected start write/read\n");

    if (playerConnected[player]) {
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

        // Zliczanie wywołań I2C
       // i2c_rw_call_count[player]++;
        
        if (result == ESP_OK) {
            // Valid I2C data received
            if (reader[0] == 0x00 && reader[1] == 0x06) {    // Correct rumble header
                if (reader[3] == 0 && reader[5] == 0) {
                    ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: processing\n");     
                        // Both rumble motors off
                    if (rumble_off_state[player]) {         //if zero data raw_fb packet has been send already, return to avoid unnecessary transfers
                        ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: RUMBLE_OFF is already true, nothing to do - leaving\n");     
                        return;
                    }
                       
                        //sending zero data raw_fp packet                 
                        struct raw_fb fb_data = {0};
                        fb_data.header.wired_id = player;
                        fb_data.header.type = FB_TYPE_RUMBLE;
                        fb_data.header.data_len = 0;  // Rumble off command
                        ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: sending packet for RUMBLE_OFF\n");    
                        adapter_q_fb(&fb_data);
                        ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: RUMBLE_OFF done, setting flag to true - avoid duplicates sending RUMBLE_OFF if still zero\n");    
                        rumble_off_state[player] = true; // set flag true                      
                    
                } else {     // At least one motor active
                    ets_printf("OGX360_PROCESS: recived MOTOR DATA\n");    
                    static uint8_t last_values[OGX360_I2C_PORT_MAX][2] = {0};
                    bool values_changed = (reader[3] != last_values[player][0] || reader[5] != last_values[player][1]);                    
                    if (values_changed || rumble_off_state[player]) {                        
                        rumble_off_state[player] = false;
                        last_values[player][0] = reader[3];
                        last_values[player][1] = reader[5];                        
                        struct raw_fb fb_data = {0};  
                        fb_data.header.wired_id = player;
                        fb_data.header.type = FB_TYPE_RUMBLE;
                        fb_data.header.data_len = 2;
                        fb_data.data[0] = reader[3];  // Left motor low
                        fb_data.data[1] = reader[5];  // Left motor high
                        adapter_q_fb(&fb_data); //send rumble packet
                    }
                }
            } else {  //wrong header

                if (rumble_off_state[player]) {
                    return;
                }
                struct raw_fb fb_data = {0};
                fb_data.header.wired_id = player;
                fb_data.header.type = FB_TYPE_RUMBLE;
                fb_data.header.data_len = 0;  // Force rumble off
                adapter_q_fb(&fb_data);
                rumble_off_state[player] = true;
                
            }
        } else {  // I2C error
            ets_printf("\x1b[31mI2C write/read failed for player %d! Error: %d\x1b[0m\n", player, result);
        }   
    }
    //uint64_t current_time = esp_timer_get_time();
   // if (current_time - last_reset_time >= 20000000) { // 20 sekund w mikrosekundach
        //for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
           // ets_printf("OGX360 Stats (last minute): Port %d: Process calls=%d, I2C R/W calls=%d\n",
           // i, process_call_count[i], i2c_rw_call_count[i]);
      //  }
        
 // Resetowanie liczników
       // memset(process_call_count, 0, sizeof(process_call_count));
       // memset(i2c_rw_call_count, 0, sizeof(i2c_rw_call_count));
       // last_reset_time = current_time;
   // }
}



/*
static void ogx360_polling_task(void) {
   
   
     bool initialized = false;
    while(1) {
        ets_printf("ogx360_polling_task: w petli while(1) \n");
        
         for (uint8_t player = 0; player < 4; player++) {
           //  bool send_data = false;
             
             // Sprawdź flagę aktualizacji
          //   portENTER_CRITICAL(&data_mutex);
            // if(data_updated[player]) {
                 //send_data = true;
                 //data_updated[player] = false;
             //}
             //portEXIT_CRITICAL(&data_mutex);
             
             //if(!send_data) continue;
             
                 ets_printf("ogx360_polling_task - pinging slaves? \n");
                 //ogx360_pingSlaves();
                 ets_printf("ogx360_polling_task -ustaw inic na true? \n");
                 //initialized = true;
                
                ets_printf("ogx360_polling_task: portactive and playernot - pinging slaves again? \n");

                if(portActive[player] != playerConnected[player]) {
                   // ogx360_pingSlaves();
                }
                ets_printf("ogx360_polling_task: if player conn write_read \n");

                if(playerConnected[player]) {
                    ets_printf("ogx360_polling_task:player connected \n");
                    uint8_t reader[6];
                    i2c_master_write_read_device(
                        I2C_NUM_0,
                     player + 1,
                     wired_adapter.data[player].output,
                     21,
                     reader,
                     sizeof(reader),
                     10
                 );
                 ets_printf("ogx360_polling_task: po write read \n");                
                 // Obsłuż błędy I2C jeśli potrzebne
                }
            }
           // vTaskDelay(pdMS_TO_TICKS(10)); // Dopasuj interwał
        }
    }
    */

   
void ogx360_i2c_init(uint32_t package) {
   
    /* 
    gpio_set_level_iram(sda_pin, 1);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG_IRAM[sda_pin], PIN_FUNC_GPIO);
    gpio_set_direction_iram(sda_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode_iram(sda_pin, GPIO_PULLUP_ONLY);
    gpio_matrix_out(sda_pin, I2CEXT0_SDA_OUT_IDX, false, false);
    gpio_matrix_in(sda_pin, I2CEXT0_SDA_IN_IDX, false);
    // Konfiguracja pinów SCL
    gpio_set_level_iram(scl_pin, 1);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG_IRAM[scl_pin], PIN_FUNC_GPIO);
    gpio_set_direction_iram(scl_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode_iram(scl_pin, GPIO_PULLUP_ONLY);
    gpio_matrix_out(scl_pin, I2CEXT0_SCL_OUT_IDX, false, false);
    gpio_matrix_in(scl_pin, I2CEXT0_SCL_IN_IDX, false);

    ets_printf("i2c_init: pins ready \n");     

    // --- Włączenie zegara i reset magistrali I2C0 ---
    //periph_module_enable(PERIPH_I2C0_MODULE);
   // periph_module_reset(PERIPH_I2C0_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_I2C0_MODULE);
    
    // Uzyskanie wskaźnika do rejestrów kontrolera
    i2c_dev_t *hw = I2C_LL_GET_HW(I2C_NUM_0);
         
    ets_printf("clk clear poszedl \n");   

    // --- Inicjalizacja w trybie master ---
    i2c_ll_master_init(hw);
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);

    // --- Disable and clear all interrupts
    i2c_ll_disable_intr_mask(hw, 0xFFFFFFFF);
    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFF);  

    //Konfiguracja prędkości (400 kHz)
    i2c_hal_clk_config_t clk_conf = {0};
    i2c_ll_master_cal_bus_clk(80 * 1000 * 1000, 400000, &clk_conf); // 80 MHz APB
    i2c_ll_master_set_bus_timing(hw, &clk_conf);   
    i2c_ll_update(hw);
    i2c_ll_master_fsm_rst(hw);
    hw->ctr.ms_mode = 1;// master mode, in i2c_ll_master_init this bit is 0 default - use after fsm reset 
    //i2c_ll_enable_controller(hw);//enable after configuration is completed
    ets_printf("I2C init: configured at 400kHz\n");*/
    
   
   
   /*
    // --- Przygotowanie danych do wysłania ---
    ets_printf("ogx360_i2c_init :  --- Ustawienie komend: RESTART, WRITE, STOP ---  \n");
    
    uint8_t buf[2];
    buf[0] = (0x01 << 1) | 0;  // adres 0x01 + bit W=0
    buf[1] = 0xAA;            // wysyłany bajt
    i2c_ll_write_txfifo(hw, buf, 2);

    // --- Ustawienie komend: RESTART, WRITE, WRITE, STOP ---
    i2c_ll_hw_cmd_t cmd = {0};
    cmd.op_code = I2C_LL_CMD_RESTART;
    cmd.byte_num = 0;
    cmd.ack_en = 0;  // oczekujemy 
    cmd.ack_exp = 0; // oczekiwany 
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);
    ets_printf("ogx360_i2c_init : RESTART POSZEDL  \n");
    
    //WRITE adress
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1);  // komenda 1: write adres
    ets_printf("ogx360_i2c_init :  WRITE ADRESS POSZEDL \n"); 

    //WRITE data
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;   
    i2c_ll_master_write_cmd_reg(hw, cmd, 2);  // komenda 2: write dane
    ets_printf("ogx360_i2c_init :  WRITE 1  BAJT POSZEDL \n");

    // STOP
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.byte_num = 0;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 3);  // komenda 3: stop
    ets_printf("ogx360_i2c_init :  KOMENDA STOP POSZLA \n");

    // --- Clear error flags before start ---
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    
    // --- Rozpoczęcie transmisji ---
    i2c_ll_master_trans_start(hw);
    ets_printf("poszlo i2c_ll_master_trans_start  \n");
 
    // --- Oczekiwanie na zakończenie transmisji (polecenie STOP) ---
    while (!i2c_ll_master_is_cmd_done(hw, 3)) {    
    // Czekamy...
    // TODO: ADD timeout/RETRY lajter for while !i2c_ll_master_is_cmd_done
    //int retries = 3;
    // while (retries--) {
    // ... [transmission code] ...
    //if (!hw->int_raw.ack_err) break;
    //ets_delay_us(100000); // 100ms delay
    //}
    }
    ets_printf("poszlo i2c_ll: master_is_cmd_done  \n");

    // sprawdzenie błędów
    // --- Critical Error Check ---
    if (hw->int_raw.ack_err) {
        ets_printf("I2C ERROR: NACK received!\n");
        // Handle error: reset FIFOs, clear bus, etc.
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        i2c_ll_master_fsm_rst(hw);  // Reset FSM
        hw->int_clr.ack_err = 1;    // Clear error flag
    }
  */  
     
     


	ets_printf("ogx360_i2c_init:  Starting dummy  \n");
    if (!initialized) {
        ets_printf("ogx360_i2c_init: not initialized - settings ports to zero  \n");
        //initialized = false;
        ogx360_i2c_port_cfg(0xf);
        ets_printf("ogx360_i2c_init: ports init zero done \n");
       
       
    } else {
        //ogx360_i2c_port_cfg(0x0);
    ets_printf("ogx360_i2c_init: already initialized dont reset ports \n");
    ets_printf("ogx360_i2c_init: starting ogx360 infinity loop \n");
   }
}


void ogx360_i2c_port_cfg(uint16_t mask) {
    ports_update_pending = true;
    ets_printf("OGX360_I2C_PORT_CFG - Updating port_cfg mask: 0x%04X\n", mask);
     
    // Pobierz wskaźnik do rejestrów I2C
    //i2c_dev_t *hw = I2C_LL_GET_HW(I2C_NUM_0);
    
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        bool was_active = portActive[i];
        portActive[i] = (mask & (1 << i)) != 0;
        
        ets_printf("OGX360_I2C_PORT_CFG: Port %d %s\n", i, portActive[i] ? "ACTIVE" : "INACTIVE");
        if (initialized) {
            ets_printf("OGX360_I2C_PORT_CFG: Checking port %d: was_active=%d, new_active=%d\n", i, was_active, portActive[i]);
        
            if (portActive[i] != was_active) {  
                ets_printf("OGX360_I2C_PORT_CFG: Change detected on port %d\n", i);
                if (portActive[i]) {  // Aktywacja portu
                    ets_printf("OGX360_I2C_PORT_CFG: Port %d ACTIVATED, attempting ping\n", i);
                    const uint8_t ping = 0xAA;
                    // const char ping[] = { 0xAA };
                    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, &ping, 1, 10);                
                    if (result == ESP_OK) {
                        ets_printf("OGX360_I2C_PORT_CFG: Pinging Arduino on port %d OK\n", i);
                        playerConnected[i] = true;
                    } else {
                        ets_printf("OGX360_I2C_PORT_CFG: \x1b[31mPinging Arduino failed! Error: %d\x1b[0m\n", result);
                        ets_printf("OGX360_I2C_PORT_CFG: Setting port %d as false\n", i);
                        portActive[i] = false;
                    }
                } else {  // Dezaktywacja portu
                    ets_printf("OGX360_I2C_PORT_CFG: Port %d DEACTIVATED, sending disconnect\n", i);
                    const uint8_t disconnect = 0xF0;
                    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, &disconnect, 1, 10);
                
                    if (result == ESP_OK) {
                        ets_printf("OGX360_I2C_PORT_CFG: Detaching Arduino USB OK\n");
                        playerConnected[i] = false;
                    } else {
                        ets_printf("OGX360_I2C_PORT_CFG: \x1b[31mDetach failed! Error: %d\x1b[0m\n", result);
                        playerConnected[i] = false; //TODO :func - GPIO output pulling LOW Arduino RST_PIN   
                    }
                }
            }
        }
    }
    ets_printf("OGX360_I2C_PORT_CFG: all done\n");
    ports_update_pending = false;
}

/*
void ogx360_i2c_port_cfg(uint16_t mask) {
    ets_printf("OGX360_I2C_PORT_CFG - Updating port_cfg mask: 0x%04X\n", mask);
    
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        bool was_active = portActive[i];
        portActive[i] = (mask & (1 << i)) != 0;
        ets_printf("OGX360_I2C_PORT_CFG: Port %d %s\n", i, portActive[i] ? "ACTIVE" : "INACTIVE");

    // Block 1: If this block runs, no other blocks should be executed. Runs at every mask update till i2c is initialized 
        if (!initialized) {
            // ets_printf("OGX360_I2C_PORT_CFG: Checking port %d: was_active=%d, new_active=%d\n", i, was_active, portActive[i]);
            if (portActive[i] != was_active) {
                ets_printf("OGX360_I2C_PORT_CFG:not init -  Change detected on port %d\n", i);
                    if (portActive[i]) {
                    ogx360_initialize_i2c();
                    ets_printf("OGX360_i2c_port_cfg: not init -  i2c initialized going to ping \n");
                    memset(playerConnected, 0, sizeof(playerConnected));
                    ogx360_pingSlaves();
                    ets_printf("OGX360_i2c_port_cfg: not init -ping done - setting initialized true \n");
                    initialized = true;
                    //ogx360_polling_task();
                   // ogx360_i2c_port_cfg(0x0);
                }
            }
            continue;
            //playerConnected[i] = false;  // Mark player as not connected
            //ets_printf("OGX360_I2C_PORT_CFG: not initialized setting Player %d  not connected on port %d\n", i, i);

        // If i2c is initialized Block 1 is not executed,  - execute  Block 2
        } else {


    // Block 2: Handle connection when i2c is initialized.
            ets_printf("OGX360_I2C_PORT_CFG: Checking port %d: was_active=%d, new_active=%d\n", i, was_active, portActive[i]);
            if (portActive[i] != was_active) {                                
                ets_printf("OGX360_I2C_PORT_CFG: Change detected on port %d\n", i);

                // If port have changed state to active                
                if (portActive[i]) {
                    ets_printf("OGX360_I2C_PORT_CFG: Port %d ACTIVATED, attempting ping\n", i);

                    // ping arduino for usb attach
                    const char ping[] = { 0xAA };
                    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)ping, sizeof(ping), 150);

                    if (result == ESP_OK) {
                        ets_printf("OGX360_I2C_PORT_CFG: Pinging Arduino on port %d OK\n", i);
                        playerConnected[i] = true;
                        // vTaskDelay(16 / portTICK_PERIOD_MS);
                        // trigger_special_rumble(i, i + 1);

                    } else {
                        ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Pinging Arduino on port %d failed with error code: %d\x1b[0m\n", i, result);
                        ets_printf("OGX360_I2C_PORT_CFG: Setting port %d as faalse \n", i);
                        portActive[i] = false;
                    }

                //If port have changed state to inactive            
                } else {
                    ets_printf("OGX360_I2C_PORT_CFG: Port %d DEACTIVATED, sending disconnect packet\n", i);
                    const char disconnectPacket[] = { 0xF0 };
                    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, (void*)disconnectPacket, sizeof(disconnectPacket), 150);
                    
                    if (result == ESP_OK) {
                        ets_printf("OGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d OK\n", i);
                        playerConnected[i] = false;

                    } else {
                        ets_printf("\x1b[31mOGX360_I2C_PORT_CFG: Detaching Arduino USB on port %d failed. Error code: %d\x1b[0m\n", i, result);
                    }
                }
            }
        }
    }
}
*/
