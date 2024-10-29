#ifndef _OGX360_I2C_H_
#define _OGX360_I2C_H_
#include <stdint.h>

#define OGX360_I2C_PORT_MAX 4
void ogx360_initialize_i2c(void);
void ogx360_pingSlaves();
void ogx360_i2c_init(uint32_t package);
void ogx360_i2c_port_cfg(uint16_t mask);
void ogx360_process(uint8_t player);

#endif /* _OGX360_I2C_H_ */



