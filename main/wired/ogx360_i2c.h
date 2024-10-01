 #ifndef _OGX360_I2C_H_
#define _OGX360_I2C_H_
#include <stdint.h>


void ogx360_initialize_i2c(void);
void ogx360_pingSlaves();

void ogx360_i2c_init(uint32_t package);
void ogx360_i2c_port_cfg(uint16_t mask);
//void print_i2c_settings(void);
//void ogx360_i2c_write(uint8_t port, uint8_t *data, size_t length);
//void ogx360_i2c_read(uint8_t port, uint8_t *data, size_t length);
// void check_aktive_ports()
void ogx360_process(uint8_t player);
void ogx360_acc_toggle_fb(uint32_t wired_id, uint16_t left_motor, uint16_t right_motor);
#endif /* _OGX360_I2C_H_ */



