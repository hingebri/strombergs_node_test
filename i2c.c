/*
 * i2c.c
 *
 *  Created on: Feb 8, 2022
 *      Author: HaraldLIngebrigtsen
 */

#include "i2c.h"

#include "em_i2c.h"
#include "em_wdog.h"

#include "rfid.h"

/** @brief Print I2C error
 *  @param code Error code from driver
 */
static void printI2CError(const int code)
{
  emberAfCorePrint("I2C error: ");

  switch (code)
  {
    case -1:
      emberAfCorePrintln("NACK received");
      break;
    case -2:
      emberAfCorePrintln("Bus error during transfer (misplaced START/STOP)");
      break;
    case -3:
      emberAfCorePrintln("Arbitration lost during transfer");
      break;
    case -4:
      emberAfCorePrintln("Usage fault");
      break;
    case -5:
      emberAfCorePrintln("SW fault");
      break;
    default:
      emberAfCorePrintln("Unknown; code = %d", code);
      break;
  }

}

static EmberStatus doTransfer(I2C_TypeDef* i2c, I2C_TransferSeq_TypeDef* seq, const uint8_t maxAttempts)
{
  int count = 0;

  // Initialize transfer
  I2C_TransferReturn_TypeDef sta = I2C_TransferInit(I2C0, seq);

  // Do transfer until done or "timeout"
  while (sta == i2cTransferInProgress && count++ < maxAttempts) {
    halCommonDelayMilliseconds(1);
    sta = I2C_Transfer(I2C0);
  }

  // Check result
  if (sta == i2cTransferDone) {
    return EMBER_SUCCESS;
  }
  else if (sta == i2cTransferInProgress) {
    emberAfCorePrintln("I2C transfer failed (max. attempts reached)");
    return EMBER_ERR_FATAL;
  }
  else {
    emberAfCorePrintln("i2cWrite failed");
    printI2CError(sta);
    return EMBER_ERR_FATAL;
  }

}

EmberStatus i2cRead(const uint8_t address, uint8_t* rbuf, const uint8_t rlen)
{
  // Transfer sequence definition
  I2C_TransferSeq_TypeDef seq;
  seq.addr = address << 1;
  seq.flags = I2C_FLAG_READ;
  seq.buf[0].data = rbuf;
  seq.buf[0].len = rlen;

  return doTransfer(I2C0, &seq, 25);

}

EmberStatus i2cWrite(const uint8_t address, uint8_t* wbuf, const uint8_t wlen)
{
  // Transfer sequence definition
  I2C_TransferSeq_TypeDef seq;
  seq.addr = address << 1;
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = wbuf;
  seq.buf[0].len = wlen;

  return doTransfer(I2C0, &seq, 25);

}

EmberStatus i2cReadRegister(const uint8_t address, const uint8_t reg, uint8_t* rbuf, const uint8_t rlen)
{
  uint8_t wbuf[1] = { reg };
  return i2cTransaction(address, wbuf, 1, rbuf, rlen);
}

EmberStatus i2cWriteRegister(const uint8_t address, const uint8_t reg, const uint8_t val)
{
  uint8_t wbuf[2] = { reg, val };
  return i2cWrite(address, wbuf, 2);
}

EmberStatus i2cTransaction(const uint8_t address, uint8_t* wbuf, const uint8_t wlen, uint8_t* rbuf, const uint8_t rlen)
{
  I2C_TransferSeq_TypeDef seq;

  seq.addr = address << 1;
  seq.flags = I2C_FLAG_WRITE_READ;
  seq.buf[0].data = wbuf;
  seq.buf[0].len = wlen;
  seq.buf[1].data = rbuf;
  seq.buf[1].len = rlen;

  return doTransfer(I2C0, &seq, 25);

}

bool i2cCheck(uint8_t addr)
{
  return (i2cWrite(addr, NULL, 0) == EMBER_SUCCESS);
}

void i2cScan(const uint8_t first, const uint8_t last)
{
  uint8_t address;
  uint8_t count = 0;

  emberAfCorePrintln("scanning for I2C devices...");

  for (uint8_t i = first; i <= last; i++) {
    WDOGn_Feed(DEFAULT_WDOG);
    halCommonDelayMilliseconds(1);
    address = 0;
    if (i2cWrite(i, NULL, 0) == EMBER_SUCCESS) {
      address = i;
    }

    if (address == 0)
      continue;

    emberAfCorePrint("found chip on address: 0x%x (%d): ", address, address);

    switch(address) {
    case 0x1d:
      emberAfCorePrintln("MMA8653FC accelerometer"); break;
    case 0x28:
      emberAfCorePrintln("MFRC522 RFID reader"); break;
    case 0x2e:
      emberAfCorePrintln("MCP4545 potmeter"); break;
    case 0x36:
      emberAfCorePrintln("MAX17043 battery fuel gauge"); break;
    case 0x39:
      emberAfCorePrintln("TSL25721FN lux meter"); break;
    case 0x40:
      emberAfCorePrintln("SHT20 humidity and temperature sensor"); break;
    case 0x43:
      emberAfCorePrintln("FXL6408 IO extender 1"); break;
    case 0x44:
      emberAfCorePrintln("FXL6408 IO extender 2"); break;
    case 0x57:
      emberAfCorePrintln("MAX30105 High-Sensitivity Optical Sensor for Smoke Detection Applications"); break;
    case 0x5a:
      emberAfCorePrintln("CCS811 IAQ Gas sensor"); break;
    case 0x60:
      emberAfCorePrintln("MPL3115A2 precision altimeter"); break;
    case 0x68:
      emberAfCorePrintln("ICM20602 gyroscope"); break;
    case 0x69:
      emberAfCorePrintln("BMG250 gyroscope"); break;
    default:
      emberAfCorePrintln("????? Unknown device"); break;
    }

    count++;

  }

  emberAfCorePrintln("found %d device(s)", count);

}

