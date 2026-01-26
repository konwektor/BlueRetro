#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "soc/dport_reg.h"
#include "esp_private/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "hal/i2c_ll.h"
#include "hal/i2c_hal.h"
#include "system/intr.h"
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
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "esp_attr.h"
#include "adapter/wired/ogx360.h"

// ========== PROSTE LOGOWANIE (zawsze włączone) ==========
//#define OGX360_LOG(format, ...) ets_printf("[OGX360] " format, ##__VA_ARGS__)

// Lub jeśli chcesz wyłączyć wszystkie logi:
// #define OGX360_LOG(format, ...) do {} while(0)


// ========== MAKRO DO LOGOWANIA ==========
#ifdef OGX360_DEBUG
    #define OGX360_LOG(format, ...) ets_printf("[OGX360] " format, ##__VA_ARGS__)
#else
    #define OGX360_LOG(format, ...) do {} while(0)
#endif



// ========== KONFIGURACJA ==========
#define OGX360_I2C_PORT_MAX         4     // Maksymalna liczba portów
#define I2C_STARTUP_DELAY_FRAMES    1500    // Opóźnienie startowe (klatki) 1500 × 4 ms ≈ 6 s
#define PING_TIMEOUT_TICKS          5     // Liczba prób pinga
#define DISCONNECT_THRESHOLD        3     // Próg błędów do rozłączenia
#define I2C_READ_DELAY_US           50    // Opóźnienie między zapisem a odczytem
#define TIMER_INTR_NUM 19


/* ================= I2C ================= */

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     21
#define I2C_FREQ_HZ     400000
#define APB_CLK_HZ      80000000
/* ================= I2C ================= */

// ========== STRUKTURY DANYCH ==========
typedef struct {
    uint8_t ping_timeout;
    uint8_t disconnect_cnt;
    bool rumble_off_state;
    uint8_t last_rumble_left;
    uint8_t last_rumble_right;
    uint32_t last_success_time;
} ogx360_port_state_t;

// ========== ZMIENNE GLOBALNE ==========

void ogx360_ll_write_periodic(void);
void ogx360_reset_port_state(int port_idx);
bool ogx360_is_port_connected(int port_idx);
uint32_t ogx360_get_last_success_time(int port_idx);

static ogx360_port_state_t port_state[OGX360_I2C_PORT_MAX] = {0};
//static volatile uint16_t frame_cnt = 0;
//static bool i2c_ready = false;
uint16_t player_connected_mask = 0;
uint16_t port_active_mask = 0;

//static bool portActive[OGX360_I2C_PORT_MAX] = {false}; 
//static bool playerConnected[OGX360_I2C_PORT_MAX] = {false};
static bool rumble_off_state[4] = {false};
static bool initialized = false;

const uint8_t pingPacket[] = {0xAA};
const uint8_t disconnectPacket[] = { 0xF0 };




   static inline esp_err_t i2c_write21_read6(i2c_port_t port, uint8_t slave_addr,
                                            const uint8_t tx21[21],
                                            uint8_t rx_buf[6], size_t *rx_len_out)
{
    if (!tx21 || !rx_buf || !rx_len_out) return ESP_ERR_INVALID_ARG;
    

    i2c_dev_t *hw = I2C_LL_GET_HW(port);
    if (!hw) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ESP_OK;

    // Small stack buffer: 1 address byte + 21 payload = 22 bytes -> safe
    uint8_t txbuf[1 + 21];
    size_t tx_bytes = 1 + 21;
    txbuf[0] = (slave_addr << 1) | 0; // 7-bit addr + W=0
    memcpy(&txbuf[1], tx21, 21);

    // Przygotowanie adresu do odczytu: [ADDR+R]
    uint8_t read_addr_byte = (slave_addr << 1) | 1; // 7-bit addr + W=1 

    // Clear/Reset before start
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    hw->int_clr.val = 0xFFFFFFFF;

    // 2. Wypełnienie TX FIFO (Adres W + 21 bajtów + Adres R)
    //  Bezpośredni zapis do rejestru FIFO jest najszybszy w ISR!!!!
    //      hw->data.val = (slave_addr << 1) | 0; // Adres Write

    // Pętla rozpisana ręcznie dla 21 bajtów (loop unrolling jest tu zbędny przy 240MHz, ale pętla jest OK)
    //      for (int i = 0; i < 21; i++) {
    //      hw->data.val = out_data[i];
    //      }    
    //      hw->data.val = (slave_addr << 1) | 1; // Adres Read


    // Write all tx bytes into FIFO
    //      static inline void i2c_ll_write_txfifo(i2c_dev_t *hw, const uint8_t *ptr, uint8_t len)
    //
    //      {
    //          uint32_t fifo_addr = (hw == &I2C0) ? 0x6001301c : 0x6002701c;
    //          for(int i = 0; i < len; i++) {
    //          WRITE_PERI_REG(fifo_addr, ptr[i]);
    //       }
    // }

    i2c_ll_write_txfifo(hw, txbuf, tx_bytes);
    i2c_ll_write_txfifo(hw, &read_addr_byte, 1); // Kolejny bajt w kolejce po restarcie

    // Build command sequence:
    // 0: START
    // 1: WRITE (tx_bytes)  <- grouped write (one write cmd if HW supports byte_num = tx_bytes)
    // 2: RESTART (for read)
    // 3: WRITE (1) - address with R bit
    // 4: READ (expect up to 6 bytes) - we will request 5
    // 5: READ last byte with NACK
    // 6: STOP
    i2c_ll_hw_cmd_t cmd = {0};
    int cmd_index = 0;

    // START
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_index++);

    // WRITE (grouped) (22 bajty: Addr_W + 21 bajtów danych)
    cmd.op_code = I2C_LL_CMD_WRITE;
    // byte_bun depends on HW/LL  - restricts byte_num range, if so
    //split into multiple WRITE commands accordingly.
    cmd.byte_num = (uint8_t)tx_bytes; // grouped write
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_index++);

    // RESTART to switch to read
    cmd.op_code = I2C_LL_CMD_RESTART;
    // clear ack settings for restart
    cmd.ack_en = 0;
    cmd.ack_exp = 0;
    cmd.byte_num = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_index++);

    // WRITE adress with bit W=1 - read request
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_index++);

    // READ: request  bytes. For read command, set byte_num to number of bytes to read.
   // LL may require setting ack bits per-read; this grouped approach works on typical ESP32 LL.
    cmd.op_code = I2C_LL_CMD_READ;
    cmd.byte_num = 6;
    // ack_en/ack_exp here are used differently for read in some LL impls; leaving ack_en=1 to generate ACK for all
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_index++);

    //READ last byte with NACK
    //IMPORTATNT: For the last byte read, we need to NACK to signal that reading is ended 
    cmd.op_code = I2C_LL_CMD_READ;
    cmd.byte_num = 1;    // czytaj ostatni bajt
    cmd.ack_en = 1;      // włącz sterowanie bitem ACK
    cmd.ack_val = 1;     // 1 oznacza wysłanie NACK (czyli "koniec danych")
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_index++);   

    // STOP
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.ack_en = 0;
    cmd.byte_num = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_index++);

    // Start transmission
    i2c_ll_start_trans(hw);
    
    
    // Wait for completion: last used command index is cmd_index - 1
    int last_cmd_index = cmd_index - 1;
    int timeout = 2000; // tune: iterations; with ets_delay_us(10) -> ~20ms
    while (!i2c_ll_master_is_cmd_done(hw, last_cmd_index) && timeout--) {
        ets_delay_us(10);
    }

    if (timeout <= 0) {
        // Timeout -> cleanup and return
        i2c_ll_master_fsm_rst(hw);
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        hw->int_clr.val = 0xFFFFFFFF;
        *rx_len_out = 0;
        return ESP_ERR_TIMEOUT;
    }

    // If NACK / ack error from hardware
    if (hw->int_raw.ack_err) {
        hw->int_clr.ack_err = 1;
        i2c_ll_master_fsm_rst(hw);
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        *rx_len_out = 0;
        return ESP_ERR_INVALID_RESPONSE; // NACK
    }
    // Read available count from FIFO status register
    int available = hw->status_reg.rx_fifo_cnt; // Bezpieczniejsze bez maski, kompilator wie lepiej

    if (available <= 0) {
        *rx_len_out = 0;
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Cap to 6
    if (available > 6) available = 6;

    // Pop available bytes
    for (int i = 0; i < available; ++i) {
        // Użycie pola 'data' jest wystarczające, nie trzeba maskować do 0xFF
        rx_buf[i] = (uint8_t)(hw->fifo_data.data); 
    }

    *rx_len_out = (size_t)available;

    // ... reszta logiki obsługi 0x00 i częściowych danych ...
    if (available == 1 && rx_buf[0] == 0x00) {
        // cleanup FIFOs / FSM state just in case
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        hw->int_clr.val = 0xFFFFFFFF;
        return ESP_ERR_INVALID_RESPONSE; // caller interprets as "no new data"
    }
    
    if (available == 6) {
        hw->int_clr.val = 0xFFFFFFFF;
        return ESP_OK;
    }
    
    hw->int_clr.val = 0xFFFFFFFF;
    return ESP_ERR_INVALID_RESPONSE;
}


static inline esp_err_t i2c_ll_master_write(i2c_port_t i2c_num, uint8_t slave_addr, const uint8_t *data, size_t data_len) {
    
    i2c_dev_t *hw = I2C_LL_GET_HW(i2c_num);

    // Clear TX FIFO & interrupts
    i2c_ll_txfifo_rst(hw);
    hw->int_clr.val = 0xFFFFFFFF;

    // Address byte + data
    uint8_t buf[1 + data_len];
    buf[0] = (slave_addr << 1) | 0;
    memcpy(&buf[1], data, data_len);

    // Fill FIFO in safe chunks - i2c_ll_master_write_cmd_reg can handle max 16 commands       

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
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    // Start transfer
    i2c_ll_start_trans(hw);
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

/*
static inline esp_err_t i2c_ll_master_write(i2c_port_t port, uint8_t slave_addr, const uint8_t *data, size_t data_len) {
    i2c_dev_t *hw = I2C_LL_GET_HW(port);
    // 1. Czyścimy FIFO i stare przerwania (zgodne z Twoim init)
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    hw->int_clr.val = 0xFFFFFFFF;
    uint8_t addr_byte = (slave_addr << 1) | I2C_MASTER_WRITE;

    // 1. Czyścimy FIFO i listę komend przed startem
    i2c_ll_txfifo_rst(hw);
    
    // 2. Ładujemy adres i dane do FIFO
    i2c_ll_write_txfifo(hw, &addr_byte, 1);
    i2c_ll_write_txfifo(hw, data, data_len);

    // 3. Budujemy listę komend
    i2c_ll_hw_cmd_t cmd = {0};
    int cmd_idx = 0;

    // START
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_idx++);

    // WRITE (adres + 21 bajtów)
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1 + data_len; // 1 (addr) + 21
    cmd.ack_en = 1;              // Sprawdzaj ACK od Slave
    cmd.ack_exp = 0;             // Oczekujemy ACK (0)

    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_idx++);

    // STOP
    cmd.op_code = I2C_LL_CMD_STOP;
    i2c_ll_master_write_cmd_reg(hw, cmd, cmd_idx++);

    // 4. Start transmisji
    hw->ctr.trans_start = 1; // albo i2c_ll_start_trans(hw);

    // 5. Czekamy na wykonanie ostatniej komendy (STOP ma indeks 2)
    int timeout = 500;
    while (!i2c_ll_master_is_cmd_done(hw, 2) && timeout--) {
        ets_delay_us(10);
    }
    // 7. Obsługa błędów
    if (hw->int_raw.ack_err) {
        hw->int_clr.ack_err = 1;
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        return ESP_ERR_INVALID_RESPONSE;     // lub ESP_FAIL
    }

    if (timeout <= 0) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}
  */ 

static inline esp_err_t i2c_ll_master_read(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data, size_t data_len)
{
    if (!data || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_dev_t *hw = I2C_LL_GET_HW(i2c_num);

    /* ---------- Reset FIFO + IRQ ---------- */
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    hw->int_clr.val = 0xFFFFFFFF;

    /* ---------- Load address (READ) ---------- */
    uint8_t addr = (slave_addr << 1) | 1;
    i2c_ll_write_txfifo(hw, &addr, 1);

    /* ---------- Commands ---------- */
    i2c_ll_hw_cmd_t cmd = {0};

    /* START / RESTART */
    cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);

    /* WRITE address */
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1);

    /* READ data */
    cmd.op_code = I2C_LL_CMD_READ;
    cmd.byte_num = data_len;
    cmd.ack_en = 1;
    cmd.ack_exp = 0;
    cmd.ack_val = 0;              // ACK for all bytes except last
    i2c_ll_master_write_cmd_reg(hw, cmd, 2);

    /* STOP */
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 3);

    /* ---------- Start transfer ---------- */
    hw->int_clr.val = 0xFFFFFFFF;
    i2c_ll_start_trans(hw);

    /* ---------- Wait ---------- */
    int timeout = 2000;
    while (!i2c_ll_master_is_cmd_done(hw, 3) && timeout--) {
        ets_delay_us(10);
    }

    /* ---------- ACK error ---------- */
    if (hw->int_raw.ack_err) {
        i2c_ll_txfifo_rst(hw);
        i2c_ll_rxfifo_rst(hw);
        i2c_ll_master_fsm_rst(hw);
        hw->int_clr.ack_err = 1;
        return ESP_FAIL;
    }

    if (timeout <= 0) {
        return ESP_ERR_TIMEOUT;
    }

    /* ---------- Read RX FIFO ---------- */
    size_t read = 0;
    while (read < data_len) {

        uint32_t fifo_cnt = 0;
        i2c_ll_get_rxfifo_cnt(hw, &fifo_cnt);
        if (fifo_cnt == 0) {
            continue;
        }
        
        size_t chunk = MIN(data_len - read, fifo_cnt);
        i2c_ll_read_rxfifo(hw, data + read, chunk);
        read += chunk;
    }
    return ESP_OK;
}

void ogx360_ll_write_periodic(void) {
    static volatile uint16_t frame_cnt = 0;
    static volatile bool i2c_ready = false;

    // 1. SPRAWDZENIE PINU (GPIO 39) POWER_SENS_PIN 
    // Odczyt bitu 7 z rejestru IN1 (piny 32-39)
    bool is_console_on = (GPIO.in1.data >> 7) & 0x1;

    if (!is_console_on) {
        // Jeśli pin jest LOW - resetujemy wszystko i wychodzimy
        frame_cnt = 0;
        i2c_ready = false;
        return; 
    }

    // 2. JEŚLI PIN JEST HIGH - SPRAWDZAMY GOTOWOŚĆ I2C
    //AFTER CONSOLE POWER ON TIME NEEDED BY ARDUINOS TO INITIALIZE
    //PINGING EARLIER CAUSING I2C BUS STALL
    if (!i2c_ready) {
        // Odliczamy wymagany czas (1500 klatek * 4ms = 6000ms)
        if (++frame_cnt < I2C_STARTUP_DELAY_FRAMES) {
            return; // Jeszcze nie minął czas startupu
        }
        i2c_ready = true; // Czas minął, ustawiamy flagę gotowości
         player_connected_mask = 0;  // Reset masek przy starcie
          OGX360_LOG("I2C ready after %d frames\n", frame_cnt);
    }
    
   

    for (int port_idx = 0; port_idx < OGX360_I2C_PORT_MAX; port_idx++) {
    const uint16_t port_bit = (1 << port_idx);
    bool active = port_active_mask & port_bit;
    bool connected = player_connected_mask & port_bit;

    // 2.1 PORT NIEAKTYWNY I NIE POŁĄCZONY - POMIŃ
    if (!active && !connected) {
        continue;
    }

    // 2.2 PING: AKTYWNY, ALE NIE POŁĄCZONY
    if (active && !connected) {
        if (port_state[port_idx].ping_timeout == 0) {

            esp_err_t err = i2c_ll_master_write(
                I2C_NUM_0,
                port_idx + 1,  // Adres I2C = port + 1
                pingPacket,
                sizeof(pingPacket)
            );

            if (err == ESP_OK) {
                player_connected_mask |= port_bit;
                port_state[port_idx].ping_timeout = 0;
                ets_printf("[OGX360] Port %d connected (ping 0xAA OK)\n", port_idx);
            } else {
                port_state[port_idx].ping_timeout = PING_TIMEOUT_TICKS;
                ets_printf("[OGX360] Port %d ping failed (0xAA): %d\n", port_idx, err);
            }

        } else {
            port_state[port_idx].ping_timeout--;
        }
        continue;
    }
    // 2.3 DISCONNECT: NIEAKTYWNY, ALE POŁĄCZONY
    if (!active && connected) {
        ets_printf("[OGX360] Port %d disconnecting (sending 0xF0)\n", port_idx);
            i2c_ll_master_write(
                I2C_NUM_0,
                port_idx + 1,
                disconnectPacket,
                sizeof(disconnectPacket)
            );
        player_connected_mask &= ~port_bit;
        port_state[port_idx].disconnect_cnt = 0;
        OGX360_LOG("PORT[%d] disconnected\n", port_idx);
        continue;
    }

        // 3. NORMALNA KOMUNIKACJA
        // 3.1 WYSŁANIE DANYCH DO KONTROLERA

        size_t rx_received = 0; // Zmienna do odebrania faktycznej liczby bajtów
        uint8_t rx_buffer[6];  // Bufor na dane odebrane
        esp_err_t write_result = i2c_write21_read6(I2C_NUM_0, port_idx + 1, (const uint8_t *)&wired_adapter.data[port_idx].output, rx_buffer, &rx_received);

    if (write_result != ESP_OK) {
        OGX360_LOG("\x1b[31mPORT[%d] write failed: %d\x1b[0m\n", port_idx, write_result);
        if (port_state[port_idx].disconnect_cnt < 255) {
            port_state[port_idx].disconnect_cnt++;
        }
        // Zliczanie błędów
        if (port_state[port_idx].disconnect_cnt >= DISCONNECT_THRESHOLD) {
            player_connected_mask &= ~port_bit;
            port_state[port_idx].disconnect_cnt = 0;
            OGX360_LOG("\x1b[31mPORT[%d] disconnected due to write errors\x1b[0m\n", port_idx);
            continue;
        } else {
            // RESET LICZNIKA BŁĘDÓW PRZY UDANYM ZAPISIE
            port_state[port_idx].disconnect_cnt = 0;
            port_state[port_idx].last_success_time = xTaskGetTickCount();
        }

        // Krótkie opóźnienie między zapisem a odczytem
        ets_delay_us(100);

        //ets_printf("OGX360_LL_write_periodic: PORT[%d] | Read data: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X [4]=0x%02X [5]=0x%02X\n", 
        //                                           i, rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4], rx_buffer[5]);
                    

        // Opcjonalne krótkie opóźnienie między zapisem a odczytem
        //ets_delay_us(100);
        // esp_err_t read_result = i2c_ll_master_read(I2C_NUM_0, i + 1, reader, 6);

        // 3.3 ODCZYT ODPOWIEDZI Z KONTROLERA
        // if (read_result == ESP_OK) {
        // Przetwarzanie danych rumble  
        if (rx_buffer[0] == 0x00 && rx_buffer[1] == 0x06) {
            if (rx_buffer[3] == 0 && rx_buffer[5] == 0) {
                // Both rumble motors off
                if (!port_state[port_idx].rumble_off_state) {
                    continue;
                }
                // 0 data - rumble stop         
                struct raw_fb fb_data = {0};
                fb_data.header.wired_id = port_idx;
                fb_data.header.type = FB_TYPE_RUMBLE;
                fb_data.header.data_len = 0;  // Rumble off command
                adapter_q_fb(&fb_data);
                port_state[port_idx].rumble_off_state = true;
                OGX360_LOG("PORT[%d] rumble motors stopped\n", port_idx);
                continue;
            }            
        } else {
            // 4.2 AKTYWACJA SILNIKÓW RUMBLE
        
            // At least one motor active
            //ets_printf("OGX360_LL_write_periodic: PORT[%d] | Read data: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X [4]=0x%02X [5]=0x%02X\n", 
            //    i, reader[0], reader[1], reader[2], reader[3], reader[4], reader[5]);
            bool values_changed = (rx_buffer[3] != port_state[port_idx].last_rumble_left || rx_buffer[5] != port_state[port_idx].last_rumble_right);

            if (values_changed || port_state[port_idx].rumble_off_state) {
                port_state[port_idx].rumble_off_state = false;
                port_state[port_idx].last_rumble_left = rx_buffer[3];
                port_state[port_idx].last_rumble_right = rx_buffer[5];
                struct raw_fb fb_data = {0};
                fb_data.header.wired_id = port_idx;
                fb_data.header.type = FB_TYPE_RUMBLE;
                fb_data.header.data_len = 2;
                fb_data.data[0] = rx_buffer[3];  // Lewy silnik
                fb_data.data[1] = rx_buffer[5];  // Prawy silnik
                adapter_q_fb(&fb_data);
                OGX360_LOG("PORT[%d] rumble updated: L=%d, R=%d\n", 
                port_idx, rx_buffer[3], rx_buffer[5]);
            }
        }
    } else {
        // 4.3 NIEPRAWIDŁOWE DANE - WYMUSZENIE WYŁĄCZENIA RUMBLE
   
        if (!port_state[port_idx].rumble_off_state) {
            continue;
        }
        struct raw_fb fb_data = {0};
        fb_data.header.wired_id = port_idx;
        fb_data.header.type = FB_TYPE_RUMBLE;
        fb_data.header.data_len = 0;
        adapter_q_fb(&fb_data);
        port_state[port_idx].rumble_off_state = true;
        OGX360_LOG("PORT[%d] forced rumble stop due to invalid data\n", port_idx);
    }
}
}

// ========== FUNKCJE POMOCNICZE ==========
void ogx360_reset_port_state(int port_idx) {
    if (port_idx >= 0 && port_idx < OGX360_I2C_PORT_MAX) {
        port_state[port_idx] = (ogx360_port_state_t){0};
        OGX360_LOG("PORT[%d] state reset\n", port_idx);
    }
}

bool ogx360_is_port_connected(int port_idx) {
    if (port_idx >= 0 && port_idx < OGX360_I2C_PORT_MAX) {
        return (player_connected_mask & (1 << port_idx)) != 0;
    }
    return false;
}

uint32_t ogx360_get_last_success_time(int port_idx) {
    if (port_idx >= 0 && port_idx < OGX360_I2C_PORT_MAX) {
        return port_state[port_idx].last_success_time;
    }
    return 0;
}
                    
/*
void ogx360_ll_write_periodic(void) {
    const uint16_t bit = (1 << i);
    uint16_t active_mask = port_active_mask;
    uint16_t connected_mask = player_connected_mask;
    bool active = active_mask & bit;
    bool connected = connected_mask & bit;
    
    
    if (!i2c_ready) {
        if (++frame_cnt < I2C_STARTUP_DELAY_FRAMES) {
            return;
        }
        i2c_ready = true;
        // opcjonalnie: wyczyść connected mask
        player_connected_mask = 0;
    }

    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        
        
        // 1. Port wyłączony i brak połączenia 
        if (!active && !connected) {
            continue;
         }
        
        // 2. Port włączony, ale brak połączenia → PING 
        if (active && !connected) {
            if (ping_timeout[i] == 0) {
                esp_err_t err = i2c_ll_master_write(
                    I2C_NUM_0,
                    i + 1,
                    pingPacket,
                    sizeof(pingPacket)
                );

            if (err == ESP_OK) {
                player_connected_mask |= bit;
                ping_timeout[i] = 0;
            } else {
                ping_timeout[i] = PING_TIMEOUT_TICKS;
            }
        } else {
            ping_timeout[i]--;
        }
        continue;
  }  
 //        3. Port wyłączony, ale był połączony → DISCONNECT 
        if (!active && connected) {
            
            //ets_printf("OGX360_LL_write_periodic:port[%d] is not active, player[%d] active - going to disconnect\n", i, i);
            i2c_ll_master_write(
                I2C_NUM_0,
                i + 1,
                disconnectPacket,
                sizeof(disconnectPacket)
            );
            player_connected_mask &= ~bit;
            continue;
        }

        // Normalna komunikacja - wysłanie danych do kontrolera
        esp_err_t write_result = i2c_ll_master_write(I2C_NUM_0, i + 1, (void*)&wired_adapter.data[i].output, 21);
        
        if (write_result != ESP_OK) {
            ets_printf("\x1b[31mOGX360_LL_write_periodic: PORT[%d] | write failed\x1b[0m: %d\n", i, write_result);
            if (++disconnect_cnt[i] >= DISCONNECT_THRESHOLD) {
                // DISCONNECT
                player_connected_mask &= ~bit;
                disconnect_cnt[i] = 0;
                continue; // Pomijamy odczyt jeśli zapis się nie powiódł
        }
        //ets_delay_us(100); 

        // Odczyt odpowiedzi z kontrolera
        uint8_t reader[6];
        esp_err_t read_result = i2c_ll_master_read(I2C_NUM_0, i + 1, reader, 6);

        if (read_result == ESP_OK) {
            // Przetwarzanie danych rumble  
            if (reader[0] == 0x00 && reader[1] == 0x06) {
                if (reader[3] == 0 && reader[5] == 0) {
                    //ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: processing\n");     
                    // Both rumble motors off
                    if (rumble_off_state[i]) {  //if 0 byte packet has been send already, return to avoid unnecessary transfers
                        continue;;
                    }
                    // 0 data -  rumble stop               
                    struct raw_fb fb_data = {0};
                    fb_data.header.wired_id = i;
                    fb_data.header.type = FB_TYPE_RUMBLE;
                    fb_data.header.data_len = 0;  // Rumble off command
                    //ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: sending packet for RUMBLE_OFF\n");    
                    adapter_q_fb(&fb_data);
                    //ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: RUMBLE_OFF done, setting flag to true - avoid duplicates sending RUMBLE_OFF if still zero\n");    
                    rumble_off_state[i] = true; // set flag true
                } else {
                    // At least one motor active
                    //ets_printf("OGX360_PROCESS: recived MOTOR DATA\n");
                    //ets_printf("OGX360_LL_write_periodic: PORT[%d] | Read data: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X [4]=0x%02X [5]=0x%02X\n", i, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);//
                    static uint8_t last_values[OGX360_I2C_PORT_MAX][2] = {0};
                    bool values_changed = (reader[3] != last_values[i][0] || reader[5] != last_values[i][1]);
                    if (values_changed || rumble_off_state[i]) {
                        rumble_off_state[i] = false;
                        last_values[i][0] = reader[3];
                        last_values[i][1] = reader[5];                        
                        struct raw_fb fb_data = {0};  
                        fb_data.header.wired_id = i;
                        fb_data.header.type = FB_TYPE_RUMBLE;
                        fb_data.header.data_len = 2;
                        fb_data.data[0] = reader[3];  // Left motor low
                        fb_data.data[1] = reader[5];  // Right motor high
                        adapter_q_fb(&fb_data); //  rumble packet
                    }
                }
            } else {
                //wrong DATA
                if (rumble_off_state[i]) {
                    continue;;
                }
                struct raw_fb fb_data = {0};
                fb_data.header.wired_id = i;
                fb_data.header.type = FB_TYPE_RUMBLE;
                fb_data.header.data_len = 0;  // Force rumble off
                adapter_q_fb(&fb_data);
                rumble_off_state[i] = true;
            }
        } else {
            // I2C failure
            ets_printf("\x1b[31mI2C write/read failed for player %d! Error: %d\x1b[0m\n", i, read_result);
        
        }
    }
}
*/

// Inicjalizacja timera na Core 1 (Bare Metal)
static void timer_init_bare_core1(void) {
    /* 1. Włącz zegar dla Timer Group 0 */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_TIMERGROUP_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_TIMERGROUP_RST);

    /* 2. Konfiguracja Timer 0 w Grupie 0 */
    TIMERG0.hw_timer[0].config.tx_en = 0; // Zatrzymaj na czas konfiguracji
    TIMERG0.hw_timer[0].config.tx_divider = 80; // Prescaler: 80MHz / 80 = 1 MHz (1 tick = 1 us)
    
    TIMERG0.hw_timer[0].loadlo.val = 0;
    TIMERG0.hw_timer[0].loadhi.val = 0;
    TIMERG0.hw_timer[0].load.val = 1;  // Załaduj wartość licznika

    /* 3. Ustawienie Alarmu na 5ms (4000 ticks * 1us = 4ms) */
    TIMERG0.hw_timer[0].alarmlo.val = 4000; 
    TIMERG0.hw_timer[0].alarmhi.val = 0;
    
    TIMERG0.hw_timer[0].config.tx_alarm_en = 1;
    TIMERG0.hw_timer[0].config.tx_autoreload = 1; // Auto-restart licznika
    TIMERG0.hw_timer[0].config.tx_level_int_en = 1; // Przerwanie typu Level
    
    /* 4. Włączenie przerwania w module Timera */
    TIMERG0.int_clr_timers.val = 1; // Wyczyść flagę dla T0 (bit 0)
    TIMERG0.int_ena_timers.val = 1; // Włącz przerwanie T0 (bit 0)

    /* 5. Start Timera */
    TIMERG0.hw_timer[0].config.tx_en = 1;
}

// Zmodyfikowany dispatcher ISR (w IRAM)
static unsigned isr_dispatch(unsigned cause) {
    // Sprawdzamy, czy bit odpowiadający TIMER_INTR_NUM jest ustawiony
    if (cause & (1 << TIMER_INTR_NUM)) {
        
        /* KRYTYCZNE: Wyczyść przerwanie w Timerze, aby uniknąć pętli */
        TIMERG0.int_clr_timers.val = 1;
        TIMERG0.hw_timer[0].config.tx_alarm_en = 1; // Wymagane w niektórych wersjach chipu po alarmie

        /* Wywołaj funkcję wysyłającą dane */
        ogx360_ll_write_periodic();
    }
    return 0;
}



/*void ogx360_i2c_ping1(uint8_t player) {
    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, player + 1, pingPacket, sizeof(pingPacket), 10);
    playerConnected[player] = (result == ESP_OK);
    ets_printf("ogx360_i2c_ping1: Player %d ping %s\n",player +1, (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");
    
}
    */

void ogx360_i2c_disconnectPacket1(uint8_t player) {
    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, player + 1, disconnectPacket, sizeof(disconnectPacket), 10);
    if (result == ESP_OK) {
        //playerConnected[player] = false;
        ets_printf("OGX360_pingSlaves: Port %d disconnected successfully\n", player +1);
    }
    else {
        ets_printf("\x1b[31mOGX360_pingSlaves: Port %d disconnect failed\x1b[0m: %d\n", player + 1, result);
        //After detach failure we need to disconnect arduino from xbox USB port!
        //TODO - implement arduino hard reset over gpio pin or wired reinit
    }     
    // playerConnected[player] = (result == ESP_OK);
     //ets_printf("ogx360_i2c_disconnectPacket1: player %d disconnectPacket %s\n",player + 1, (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");
} 

void ogx360_i2c_ping_ll(void) { //LL PING WITH HARDCODED ADDRES AND DATA - TESTING - TO MOVE INTO SEPARATE CONF
    i2c_dev_t *hw = &I2C0;
    //i2c_dev_t *hw = I2C_LL_GET_HW(i2c_num);
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll --- Setting addres and commands: RESTART, WRITE, STOP ---  \n");
    
    uint8_t buf[2];
    buf[0] = (0x01 << 1) | 0;  // addres 0x01 + bit W=0
    buf[1] = 0xAA;            // byte to send
    i2c_ll_write_txfifo(hw, buf, 2);

    // --- Commands set: RESTART, WRITE, WRITE, STOP ---
    i2c_ll_hw_cmd_t cmd = {0};
    cmd.op_code = I2C_LL_CMD_RESTART;
    cmd.byte_num = 0;
    cmd.ack_en = 0;  // oczekujemy 
    cmd.ack_exp = 0; // oczekiwany 
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 0);
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll : RESTART DONE  \n");
    
    //WRITE adress
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 1);  // COMMAND 1: write addres
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll : WRITE ADDRES DONE \n"); 

    //WRITE data
    cmd.op_code = I2C_LL_CMD_WRITE;
    cmd.byte_num = 1;
    cmd.ack_en = 1;  // oczekujemy ACK od slave'a
    cmd.ack_exp = 0; // oczekiwany ACK (0)
    cmd.ack_val = 0;
    cmd.done = 0;   
    i2c_ll_master_write_cmd_reg(hw, cmd, 2);  // COMMAND 2: write data
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll :  WRITE 1  BAJT POSZEDL \n");

    // STOP
    cmd.op_code = I2C_LL_CMD_STOP;
    cmd.byte_num = 0;
    cmd.ack_en = 0;
    i2c_ll_master_write_cmd_reg(hw, cmd, 3);  // COMMAND 3: stop
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll :  KOMENDA STOP POSZLA \n");

    // --- Clear error flags before start ---
    hw->int_clr.val = 0xFFFFFFFF;  // Clear all pending interrupts
    
    // --- Rozpoczęcie transmisji ---
   i2c_ll_start_trans(hw);
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll : SEND i2c_ll_master_trans_start  \n");
 
    // --- Oczekiwanie na zakończenie transmisji (polecenie STOP) ---
    while (!i2c_ll_master_is_cmd_done(hw, 3)) {
    	// TODO: ADD timeout/RETRY
    	// WAITIN...
    	//int retries = 3;
    	// while (retries--) {
    	// ... [transmission code] ...
    	//if (!hw->int_raw.ack_err) break;
    	//ets_delay_us(100000); // 100ms delay
    	//}
    }
    ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll : i2c_ll: master_is_cmd_done  \n");

    
    // --- Critical Error Check ---
    if (hw->int_raw.ack_err) {
        ets_printf("ogx360_LOW_LEVEL_i2c_ping_ll I2C ERROR: NACK received!\n");
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



/*void ogx360_initialize_i2c(void) {
    static bool i2c_done = false;

     ets_printf("OGX360_INITIALIZE_I2C - starting\n");    
    if  (i2c_done) { //If wired_adapter_reinit - dont initialize  already initialized i2c!
        ets_printf("OGX360_INITIALIZE_I2C - don`t initialize  already initialized i2c - returning from function!\n");
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
       
        if (I2C0.timeout.tout != 0) {
            i2c_set_timeout(I2C_NUM_0, 1048575);
        }
        ets_printf("OGX360_INITIALIZE_I2C - I2C initialization complete.\n");
        i2c_done = true;
        initialized =true;
    }
}
    */

/*void ogx360_pingSlaves() {
	for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
    
}
    */


/*
void ogx360_ll_process(uint8_t player)
{
    const uint16_t bit = (1 << player);

    uint16_t active_mask    = port_active_mask;
    uint16_t connected_mask = player_connected_mask;

    bool active    = active_mask    & bit;
    bool connected = connected_mask & bit;

    // 1. Port wyłączony i brak połączenia //
    if (!active && !connected) {
        ets_printf("OGX360_LL_PROCESS:port[%d] , player[%d] not active quitting\n", player, player);
        return;
    }

    // 2. Port włączony, ale brak połączenia → PING //
    if (active && !connected) {
        ets_printf("OGX360_LL_PROCESS:port[%d] is active , player[%d] not - going ping \n", player, player);
        esp_err_t err = i2c_ll_master_write(
            I2C_NUM_0,
            player + 1,
            pingPacket,
            sizeof(pingPacket)
        );

        if (err == ESP_OK) {
            player_connected_mask |= bit;
        }
        return;
    }

    // 3. Port wyłączony, ale był połączony → DISCONNECT //
    if (!active && connected) {
         ets_printf("OGX360_LL_PROCESS:port[%d] is not active, player[%d] active - going to disconnect \n", player, player);
        i2c_ll_master_write(
            I2C_NUM_0,
            player + 1,
            disconnectPacket,
            sizeof(disconnectPacket)
        );

        player_connected_mask &= ~bit;
        return;
    }

    esp_err_t result = i2c_ll_master_write(
        I2C_NUM_0,
        player + 1,
        (void*)&wired_adapter.data[player].output,
        21
    );
    if (result != ESP_OK) {
       ets_printf("\x1b[31mOGX360_LL_PROCESS: write failed\x1b[0m: %d\n", result);
    }

    
    //if (result != ESP_OK) {
        // slave nie odpowiada → uznaj za rozłączony //
    //    player_connected_mask &= ~bit;
    //}
}
*/
/*
void ogx360_ll_process(uint8_t player) {
    //if (!initialized) {
    //    ets_printf("OGX360_LL_PROCESS: i2c not initialized\n");
     //   ogx360_initialize_i2c();
    //}
   if (!portActive[player]) {
        ets_printf("OGX360_LL_PROCESS:port %d is not active quitting\n", player);
         return;
    }
   if (ports_update_pending) {
        ets_printf("OGX360_LL_PROCESS:ports are updating -  quitting\n");
        return;
    }    
   if (write_pending) {
        ets_printf("OGX360_LL_PROCESS: write pending -  quitting\n");
        return;
    }
    write_pending = true;

    esp_err_t result = i2c_ll_master_write(
        I2C_NUM_0,
        player + 1,
        (void*)&wired_adapter.data[player].output,
        21
    );
    if (result != ESP_OK) {
       ets_printf("\x1b[31mOGX360_LL_PROCESS: write failed\x1b[0m: %d\n", result);
    }
    write_pending = false;


    //ets_printf("ogx360_LL_i2c_write:  %s\n", (result == ESP_OK) ? "successful" : "\x1b[31mfailed\x1b[0m");

}
*/

/*
void ogx360_process(uint8_t player) {
    //no i2c operations during ports update -else WD timeouts coz of delay during pinging devices
    if (ports_update_pending) {
       // ets_printf("OGX360_PROCESS:ports are updating -  quitting\n");
        return;
    }    
    if (!portActive[player]) {
       // ets_printf("OGX360_PROCESS:port %d is not active quitting\n", player);
         return;
    }
    if (!initialized) {
   // ets_printf("OGX360_PROCESS: i2c not initialized\n");
    ogx360_initialize_i2c();       
    return;
    }

    if (portActive[player] != playerConnected[player]) {
        ets_printf("OGX360_PROCESS: player not connected, goin to ping slave\n");
        ogx360_pingSlaves();
        ets_printf("OGX360_PROCESS: ping slaves done - quit\n");     
        return;
    }
	
	uint8_t reader[6]= {0};
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

               
        if (result == ESP_OK) {
            // Valid I2C data received
            if (reader[0] == 0x00 && reader[1] == 0x06) {    // Correct rumble header
                if (reader[3] == 0 && reader[5] == 0) {
                    //ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: processing\n");     
                    // Both rumble motors off
                    if (rumble_off_state[player]) {         //if 0 byte packet has been send already, return to avoid unnecessary transfers
                        //ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: RUMBLE_OFF is already true, nothing to do - leaving\n");     
                        return;
                    }                       
                        // 0 data -  rumble stop               
                        struct raw_fb fb_data = {0};
                        fb_data.header.wired_id = player;
                        fb_data.header.type = FB_TYPE_RUMBLE;
                        fb_data.header.data_len = 0;  // Rumble off command
                        //ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: sending packet for RUMBLE_OFF\n");    
                        adapter_q_fb(&fb_data);
                        //ets_printf("OGX360_PROCESS: ZERO MOTOR DATA: RUMBLE_OFF done, setting flag to true - avoid duplicates sending RUMBLE_OFF if still zero\n");    
                        rumble_off_state[player] = true; // set flag true                      
                    
                } else {     // At least one motor active
                    //ets_printf("OGX360_PROCESS: recived MOTOR DATA\n");    
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
                        fb_data.data[1] = reader[5];  // Right motor high
                        adapter_q_fb(&fb_data); //  rumble packet
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
        }
            else {  // I2C error
            ets_printf("\x1b[31mI2C write/read failed for player %d! Error: %d\x1b[0m\n", player, result);
            
        }   
    }
}
   */

   
void ogx360_i2c_init(uint32_t package) {

    // i2c_dev_t *hw = I2C_LL_GET_HW(i2c_num);
    
   
    /* ---------- Enable clock + reset ---------- */
    // Bezpośrednie użycie rejestrów (uwaga: może nie być atomowe!)
    if (I2C_PORT == 0) {
        // Włącz zegar dla I2C0
        SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT0_CLK_EN);
        // Zresetuj I2C0
        SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT0_RST);
        CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT0_RST);
    } else if (I2C_PORT == 1) {
        // Włącz zegar dla I2C1
        SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2C_EXT1_CLK_EN);
        // Zresetuj I2C1
        SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT1_RST);
        CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2C_EXT1_RST);
    }
     // Krótkie opóźnienie dla stabilizacji
    ets_delay_us(10);

    // KONIEC sekcji krytycznej
    /* ---------- GPIO ---------- */
    gpio_set_level_iram(I2C_SDA_PIN, 1);
    gpio_set_level_iram(I2C_SCL_PIN, 1);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG_IRAM[I2C_SDA_PIN], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG_IRAM[I2C_SCL_PIN], PIN_FUNC_GPIO);

    gpio_set_direction_iram(I2C_SDA_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction_iram(I2C_SCL_PIN, GPIO_MODE_INPUT_OUTPUT_OD);

    gpio_set_pull_mode_iram(I2C_SDA_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode_iram(I2C_SCL_PIN, GPIO_PULLUP_ONLY);

    gpio_matrix_out(I2C_SDA_PIN, I2CEXT0_SDA_OUT_IDX, false, false);
    gpio_matrix_in (I2C_SDA_PIN, I2CEXT0_SDA_IN_IDX, false);

    gpio_matrix_out(I2C_SCL_PIN, I2CEXT0_SCL_OUT_IDX, false, false);
    gpio_matrix_in (I2C_SCL_PIN, I2CEXT0_SCL_IN_IDX, false);

     i2c_dev_t *hw = I2C_LL_GET_HW(I2C_NUM_0);

    /* Disable interrupts before master init */
   
    hw->int_ena.val &= ~I2C_LL_INTR_MASK;
    
    /* ---------- Clear interrupts ---------- */
    hw->int_clr.val = I2C_LL_INTR_MASK;



    /* ---------- Master init ---------- */
    i2c_ll_master_init(hw);

    /* ---------- Clock / timing ---------- */
    i2c_hal_clk_config_t clk_cfg;
    i2c_ll_master_cal_bus_clk(APB_CLK_HZ, I2C_FREQ_HZ, &clk_cfg);
    i2c_ll_master_set_bus_timing(hw, &clk_cfg);
    i2c_ll_master_set_filter(hw, 7);

    /* ---------- FIFO reset ---------- */
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);

    

    /* ---------- Timeout ---------- */
    //i2c_ll_set_tout(hw, I2C_LL_MAX_TIMEOUT);
    i2c_ll_set_tout(hw, 32000);
    
    /* ---------- FIFO mode ---------- */
    i2c_ll_enable_fifo_mode(hw, true);

        /* ---------- Ready ---------- */
    timer_init_bare_core1();
    



    /* ZMIANA: Alokacja przerwania Timera zamiast I2C */
    // Mapujemy źródło TG0_T0 (Timer Group 0, Timer 0) do numeru TIMER_INTR_NUM
    intexc_alloc_iram(ETS_TG0_T0_LEVEL_INTR_SOURCE, TIMER_INTR_NUM, isr_dispatch);

    // Usunięto stare mapowanie I2C, ponieważ teraz Timer steruje wysyłaniem.

    // Dodatkowa konfiguracja portów jeśli potrzeba (podobnie jak w Wii).
    // ogx360_i2c_port_cfg(0x3);  // Dostosuj jeśli istnieje taka funkcja.

   
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
    
    // --- Disable and clear all interrupts
    i2c_ll_disable_intr_mask(hw, 0xFFFFFFFF);
    i2c_ll_clear_intr_mask(hw, 0xFFFFFFFF);  

    //Konfiguracja prędkości (400 kHz)
    i2c_hal_clk_config_t clk_conf = {0};
    i2c_ll_master_cal_bus_clk(80 * 1000 * 1000, 400000, &clk_conf); // 80 MHz APB
    i2c_ll_master_set_bus_timing(hw, &clk_conf); 
    //i2c_ll_master_set_filter(hw, 7);
  
    i2c_ll_set_tout(hw, 20);
    // 5. Reset FIFO i FSM
    i2c_ll_txfifo_rst(hw);
    i2c_ll_rxfifo_rst(hw);
    //i2c_ll_update(hw);
    //i2c_ll_master_fsm_rst(hw);

    // 6. Ustawienie trybu Master (Wymagane po fsm_rst)
    hw->ctr.ms_mode = 1;      // Master Mode
    hw->ctr.clk_en = 1;       // Włączenie wewnętrznego zegara bramkowania
    
    // 7. ZATWIERDZENIE (Update)
    

    ets_printf("I2C init: ESP32 WROOM-32D configured at 400kHz with Filters\n");  
      
   
   
   
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
    i2c_ll_start_trans(hw);
    ets_printf("ogx360_i2c_init :poszlo i2c_ll_master_trans_start  \n");
 
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
     
     


	//ets_printf("ogx360_i2c_init:  Starting dummy  \n");
    if (!initialized) {
        //ets_printf("ogx360_i2c_init: not initialized - settings ports to zero  \n");
        //initialized = false;
         ets_printf("ogx360_i2c_init: setting ports to 0 \n");
        ogx360_i2c_port_cfg(0x0);
        ets_printf("ogx360_i2c_init: ports init zero done \n");
       
       
    } else {
        ogx360_i2c_port_cfg(0x0);
    ets_printf("ogx360_i2c_init: already initialized dont reset ports \n");
    
   }
}

void ogx360_i2c_port_cfg(uint16_t mask)
{
   // i2c_reset_startup_state();
    ets_printf("OGX360_I2C_PORT_CFG - Updating port_cfg mask: 0x%04X\n", mask);
    port_active_mask = mask;
}

/*
void ogx360_i2c_port_cfg(uint16_t mask) {
    ports_update_pending = true;
    ets_printf("OGX360_I2C_PORT_CFG - Updating port_cfg mask: 0x%04X\n", mask);
   
    
    for (int i = 0; i < OGX360_I2C_PORT_MAX; i++) {
        bool was_active = portActive[i];
        portActive[i] = (mask & (1 << i)) != 0;
        
        ets_printf("OGX360_I2C_PORT_CFG: Port %d %s\n", i, portActive[i] ? "ACTIVE" : "INACTIVE");
        
        if (initialized) {

            ets_printf("OGX360_I2C_PORT_CFG: Checking port %d: was_active=%d, new_active=%d\n", i, was_active, portActive[i]);
            if (portActive[i] != was_active) {

                ets_printf("OGX360_I2C_PORT_CFG: Change detected on port %d\n", i);
                if (portActive[i]) {  
                    ets_printf("OGX360_I2C_PORT_CFG: Port %d ACTIVATED, attempting ping\n", i);
                    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, ping, sizeof(ping), 10);                
                    if (result == ESP_OK) {                        
                        ets_printf("OGX360_I2C_PORT_CFG: Pinging Arduino on port %d OK\n", i);
                        playerConnected[i] = true;                        
                    } else {
                        ets_printf("OGX360_I2C_PORT_CFG: \x1b[31mPinging Arduino failed! Error: %d\x1b[0m\n", result);
                        //ets_printf("OGX360_I2C_PORT_CFG: Setting port %d as false\n", i);
                        playerConnected[i] = false;
                    }
                } else { 
                   // i2c_ll_master_write(I2C_NUM_0, i + 1, disconnectPacket, sizeof(disconnectPacket));
                    
                    ets_printf("OGX360_I2C_PORT_CFG: Port %d DEACTIVATED, sending disconnect\n", i);

                    esp_err_t result = i2c_master_write_to_device(I2C_NUM_0, i + 1, disconnectPacket, sizeof(disconnectPacket), 10);                             
                    if (result == ESP_OK) {
                        ets_printf("OGX360_I2C_PORT_CFG: Detaching Arduino USB OK\n");
                        playerConnected[i] = false;
                    } else {
                        ets_printf("OGX360_I2C_PORT_CFG: \x1b[31mDetach failed! Error: %d\x1b[0m\n", result);
                        playerConnected[i] = true; //TODO :func - GPIO output pulling LOW Arduino RST_PIN   
                    }
                }
            }
        }
    }
    ets_printf("OGX360_I2C_PORT_CFG: all done\n");
    ports_update_pending = false;
}
*/
