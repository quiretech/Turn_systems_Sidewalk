#ifndef U_I2C_TURN_H
#define U_I2C_TURN_H

#include <stdio.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

uint8_t     i2c_ready(void);
uint8_t     i2c_scan(void);
bool        check_i2c_address(uint8_t addr);
uint8_t     u_i2c_write_read(uint16_t addr, uint8_t *tx_data, uint8_t tx_data_len, uint8_t *rx_data, uint8_t rx_data_len);
uint8_t     u_i2c_write(uint16_t addr, uint8_t *tx_data, uint8_t tx_data_len);
uint8_t     u_i2c_read(uint16_t addr, uint8_t *rx_data, uint8_t rx_data_len);

#endif //U_I2C_TURN_H