#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "comm_bus.h"

void i2c_bus_init();
void i2c_bus_write_byte(uint8_t data);
uint8_t i2c_bus_read_byte();
void i2c_bus_start();
void i2c_bus_stop();

CommBus *get_i2c_bus();

#endif
