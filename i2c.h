/*
 * i2c.h
 *
 *  Created on: Feb 8, 2022
 *      Author: HaraldLIngebrigtsen
 */

#ifndef I2C_H_
#define I2C_H_

#include "app/framework/include/af.h"

EmberStatus i2cRead(uint8_t address, uint8_t* wbuf, uint8_t wlen);
EmberStatus i2cWrite(uint8_t address, uint8_t* wbuf, uint8_t wlen);
EmberStatus i2cReadRegister(uint8_t address, uint8_t reg, uint8_t* rbuf, uint8_t rlen);
EmberStatus i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t val);
EmberStatus i2cTransaction(uint8_t address, uint8_t* wbuf, uint8_t wlen, uint8_t* rbuf, uint8_t rlen);

bool i2cCheck(uint8_t addr);
void i2cScan(uint8_t first, uint8_t last);

#endif /* I2C_H_ */
