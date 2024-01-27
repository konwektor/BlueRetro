#ifndef _OGX360_I2C_H_
#define _OGX360_I2C_H_

#include <stdint.h>


//void ogx360_check_connected_controllers();
//void ogx360_i2c_port_cfg(mask);
void ogx360_i2c_port_cfg(uint16_t mask);
void ogx360_i2c_init(void);

void ogx360_process(uint8_t player);

#endif /* _OGX360_I2C_H_ */
