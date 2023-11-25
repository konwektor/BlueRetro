#ifndef _OGX360_I2C_H_
#define _OGX360_I2C_H_

void ogx360_check_connected_controllers();
void ogx360_check_connected_controller(uint8_t player);
void ogx360_initialize_i2c(void);
void ogx360_process(uint8_t player, int32_t type);
void ogx360_acc_toggle_fb(uint32_t wired_id, uint8_t left_motor, uint8_t right_motor);

#endif /* _OGX360_I2C_H_ */
