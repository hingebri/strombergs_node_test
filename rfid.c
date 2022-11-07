/*
 * rfid.c
 *
 *  Created on: Feb 7, 2022
 *      Author: HaraldLIngebrigtsen
 */

/*
  Ported (and modified): https://github.com/adafruit/Adafruit_MFRC630
*/

#include "rfid.h"

#include "em_wdog.h"

#include "i2c.h"

extern uint8_t rfidAddress;

#define MAX_BUF_SIZE        32

#define LPCD_THRESHOLD_HIGH 3
#define LPCD_THRESHOLD_LOW  3

uint8_t antcfg_iso14443a_106[18] = { 0x8e, 0x12, 0x39, 0x0a, 0x18, 0x18,
                                     0x0f, 0x21, 0x00, 0xc0, 0x12, 0xcf,
                                     0x00, 0x04, 0x90, 0x5c, 0x12, 0x0a };

uint8_t LPCD_QMin = 0, LPCD_QMax = 0, LPCD_IMin = 0;

/*
void rfidHardReset()
{
  enableRfid(true);
  halCommonDelayMilliseconds(1);
  enableRfid(false);
  halCommonDelayMilliseconds(1);
  enableRfid(true);
}
*/

void rfidSoftReset()
{
  emberAfCorePrintln("soft reset");

  // Perform SW reset, then idle
  writeCommand(MFRC630_CMD_SOFTRESET);
  halCommonDelayMilliseconds(50);
  writeCommand(MFRC630_CMD_IDLE);

  // Disable IRQ sources
  write8(MFRC630_REG_IRQ0, 0x7f);
  write8(MFRC630_REG_IRQ1, 0x7f);
  write8(MFRC630_REG_IRQ0EN, 0x00);
  write8(MFRC630_REG_IRQ1EN, 0x00);

  // Flush FIFO
  write8(MFRC630_REG_FIFO_CONTROL, 0xb0);

}

void writeCommand(uint8_t command)
{
  writeBuffer(MFRC630_REG_COMMAND, 1, &command);
}

void writeParamCommand(uint8_t command, uint8_t paramlen, uint8_t *params)
{
  /* Arguments and/or data necessary to process a command are exchanged via
     the FIFO buffer:
     - Each command that needs a certain number of arguments will start
       processing only when it has received the correct number of arguments
       via the FIFO buffer.
     - The FIFO buffer is not cleared automatically at command start. It is
       recommended to write the command arguments and/or the data bytes into
       the FIFO buffer and start the command afterwards.
     - Each command may be stopped by the host by writing a new command code
       into the command register e.g.: the Idle-Command. */

  /* Cancel any current command. */
  write8(MFRC630_REG_COMMAND, MFRC630_CMD_IDLE);

  /* Flush the FIFO */
  clearFIFO();

  /* Write data to the FIFO */
  writeFIFO(paramlen, params);

  /* Send the command */
  write8(MFRC630_REG_COMMAND, command);

}

void writeBuffer(uint8_t reg, uint16_t len, uint8_t *buf)
{
  uint8_t wb[MAX_BUF_SIZE];
  uint8_t wlen = len + 1;

  if (wlen > MAX_BUF_SIZE) {
    emberAfCorePrintln("ERROR: BUF TOO BIG %d!!!", wlen);
    return;
  }

  wb[0] = reg;
  memcpy(&wb[1], buf, len);

  i2cWrite(rfidAddress, wb, wlen);

}

void write8(uint8_t reg, uint8_t value)
{
  i2cWriteRegister(rfidAddress, reg, value);
}

uint8_t read8(uint8_t reg)
{
  uint8_t res;
  i2cReadRegister(rfidAddress, reg, &res, 1);
  return res;
}

static void getWindowValues()
{
  // Get I and Q calues
  uint8_t iVal = read8(MFRC630_REG_LPCD_I_RESULT);
  uint8_t qVal = read8(MFRC630_REG_LPCD_Q_RESULT);

  // Use recommended threshold value
  //const uint8_t threshold = 1;

  // Calculate detection window values
  uint8_t bQMin = qVal - LPCD_THRESHOLD_LOW;
  uint8_t bQMax = qVal + LPCD_THRESHOLD_HIGH;
  uint8_t bIMin = iVal - LPCD_THRESHOLD_LOW;
  uint8_t bIMax = iVal + LPCD_THRESHOLD_HIGH;

  /*
  bQMin = 18;
  bQMax = 18;
  bIMin = 22;
  bIMax = 22;
  */

  emberAfCorePrintln("bQMin = %d, bQMax = %d, bIMin = %d, bIMax = %d", bQMin, bQMax, bIMin, bIMax);

  // Calculate register values
  LPCD_QMin = bQMin | ((bIMax & 0b00110000) << 2);
  LPCD_QMax = bQMax | ((bIMax & 0b00001100) << 4);
  LPCD_IMin = bIMin | ((bIMax & 0b00000011) << 6);

/*
  LPCD_QMin = 145;
  LPCD_QMax = 19;
  LPCD_IMin = 34;
*/

  /*
  LPCD_QMin = 0xff;
  LPCD_QMax = 0xff;
  LPCD_IMin = 0xff;
  */

  emberAfCorePrintln("LPCD_QMin = %d, LPCD_QMax = %d, LPCD_IMin = %d", LPCD_QMin, LPCD_QMax, LPCD_IMin);

}

void printQIValues(void)
{
  // Get I and Q calues
  uint8_t iVal = read8(MFRC630_REG_LPCD_I_RESULT);
  uint8_t qVal = read8(MFRC630_REG_LPCD_Q_RESULT);

  // Calculate detection window values
  uint8_t bQMin = qVal - LPCD_THRESHOLD_LOW;
  uint8_t bQMax = qVal + LPCD_THRESHOLD_HIGH;
  uint8_t bIMin = iVal - LPCD_THRESHOLD_LOW;
  uint8_t bIMax = iVal + LPCD_THRESHOLD_HIGH;

  emberAfCorePrintln("bQMin = %d, bQMax = %d, bIMin = %d, bIMax = %d", bQMin, bQMax, bIMin, bIMax);

}

static void getWindowValuesHPG()
{
  // Get I and Q calues
  uint8_t iVal = read8(MFRC630_REG_LPCD_I_RESULT);
  uint8_t qVal = read8(MFRC630_REG_LPCD_Q_RESULT);
  emberAfCorePrintln("ival %d, qval = %d ", iVal, qVal);

  // Use recommended threshold value
  const uint8_t threshold = 3;

  // Calculate detection window values
  uint8_t bQMin = 0;
  if(qVal >= qVal - threshold){             //check for wrapping
      bQMin = qVal - threshold;
  }
  uint8_t bQMax = 255;
  if(qVal <= qVal + threshold){             //check for wrapping
      bQMax = qVal + threshold;
  }

  uint8_t bIMin = 0;
  if(iVal >= iVal - threshold){             //check for wrapping
      bIMin = iVal - threshold;
  }
  uint8_t bIMax = 255;
  if(iVal <= iVal + threshold){             //check for wrapping
      bIMax = iVal + threshold;
  }
//  uint8_t bIMax = iVal + threshold;

//  emberAfCorePrintln("bqMin = %d, bqMax = %d, biMin = %d, biMax = %d ", bQMin, bQMax, bIMin, bIMax);

  // Calculate register values
  LPCD_QMin = bQMin | ((bIMax & 0b00110000) << 2);
  LPCD_QMax = bQMax | ((bIMax & 0b00001100) << 4);
  LPCD_IMin = bIMin | ((bIMax & 0b00000011) << 6);

  emberAfCorePrintln("LPCD_QMin = %d, LPCD_QMax = %d, LPCD_IMin = %d", LPCD_QMin, LPCD_QMax, LPCD_IMin);

}

void rfidLpcdInitHPG(void)
{
  // Perform SW reset, then idle
  getWindowValuesHPG();
  rfidSoftReset();
  //halCommonDelayMilliseconds(100);
  //update window values
  write8(MFRC630_REG_LPCD_QMIN, LPCD_QMin);
  write8(MFRC630_REG_LPCD_QMAX, LPCD_QMax);
  write8(MFRC630_REG_LPCD_IMIN, LPCD_IMin);

  // LPCD (low power card detect) config

  // Write T3 reload value(hi, lo)
  write8(MFRC630_REG_T3_RELOAD_HI, 0x07);   // Write default T3 reload value Hi
  write8(MFRC630_REG_T3_RELOAD_LO, 0xF2);   // Write default T3 reload value Lo

  // Write T4 reload value
  write8(MFRC630_REG_T4_RELOAD_HI, 0x00);   // Write min. T4 reload value Hi
  write8(MFRC630_REG_T4_RELOAD_LO, 0x13);   // Write min. T4 reload value Lo

  // Configure T4 for AutoLPCD and AutoRestart/Autowakeup.Use 2Khz LFO, Start T4
  write8(MFRC630_REG_T4_CONTROL, 0xdf);     // Config T4 for AutoLPCD & AutoRestart. Set AutoTrimm bit. Start T4.

  // Clear LPCD result
  write8(MFRC630_REG_LPCD_Q_RESULT, 0x40);

  // Set Mix2Adc bit
  write8(MFRC630_REG_RCV, 0x52);

  //******Backup current RxAna setting
  //******GR  39    // Response:  00

  // Raise receiver gain to maximum
  write8(MFRC630_REG_RX_ANA, 0x03);         // Raise receiver gain to maximum

  // Wait until T4 is started
  while(read8(MFRC630_REG_T4_CONTROL) != 0x9f) {};

  // Flush cmd and FIFO. Clear all IRQ flags
  write8(MFRC630_REG_COMMAND, 0x00);
  write8(MFRC630_REG_FIFO_CONTROL, 0xb0);
  write8(MFRC630_REG_IRQ0, 0b01111111); //clear interrupts
  write8(MFRC630_REG_IRQ1, 0b01111111); //clear interrupts
  halCommonDelayMilliseconds(100);
  write8(MFRC630_REG_IRQ0, 0b11110000); //set IRQ0 set/clear|HiAlertIRQ|LoAlertIRQ|IdleIRQ  |TxIRQ    |RxIRQ    |ErrIRQ   |RxSOFIrq            ***set IRQ0 interrupts
  write8(MFRC630_REG_IRQ1, 0b11100000); //set IRQ1 set/clear|GlobalIRQ |LPCD_IRQ  |Timer4IRQ|Timer3IRQ|Timer2IRQ|Timer1IRQ|Timer0IRQ           ***set IRQ1 interrupts
  // Enable IRQ sources: Idle and PLCD
  write8(MFRC630_REG_IRQ0EN, 0b01110000);  //IRQ_Inv    |Hi AlertIRQEn|Lo AlertIRQEn|IdleIRQEn  |TxIRQEn    |RxIRQEn    |ErrIRQEn   |RxSOFIRQEn          ***set IRQ0En interrupts
  write8(MFRC630_REG_IRQ1EN, 0b01110000);  //IRQPushPull|IRQPinEN     |LPCD_IRQEN   |Timer4IRQEn|Timer3IRQEn|Timer2IRQEn|Timer1IRQEn|Timer0IRQEn         ***set IRQ1En interrupts
  //set/clear  |GlobalIRQ    |LPCD_IRQ     |Timer4IRQ  |Timer3IRQ  |Timer2IRQ  |Timer1IRQ  |Timer0IRQ           ***set IRQ1 interrupts

  //> Start RC663 cmd "Low power card detection". Enter PowerDown mode.
  //write8(MFRC630_REG_COMMAND, 0b10000001);

  // Start RC663 cmd "Low power card detection". Enter PowerDown mode.
//  writeCommand(MFRC630_CMD_IDLE);
  writeCommand(MFRC630_CMD_LPCD);

}

void rfidInit() {
  writeBuffer(MFRC630_REG_DRV_MOD, sizeof(antcfg_iso14443a_106), antcfg_iso14443a_106);
  write8(MFRC630_REG_DRV_MOD, 0x8E); /* Driver mode register */
  write8(MFRC630_REG_TX_AMP, 0x12);  /* Transmitter amplifier register */
  write8(MFRC630_REG_DRV_CON, 0x39); /* Driver configuration register */
  write8(MFRC630_REG_TXL, 0x06);     /* Transmitter register */
}

void rfidLpcdInit(void)
{
  // Perform SW reset, then idle
  rfidSoftReset();

  // LPCD (low power card detect) config
  write8(MFRC630_REG_LPCD_QMIN, 0xc0);      // Set Qmin register
  write8(MFRC630_REG_LPCD_QMAX, 0xff);      // Set Qmax register
  write8(MFRC630_REG_LPCD_IMIN, 0xc0);      // Set Imin register
  write8(MFRC630_REG_DRV_MOD, 0x89);        // Set DrvMode register

  // Execute trimming procedure
  write8(MFRC630_REG_T3_RELOAD_HI, 0x00);   // Write default T3 reload value Hi
  write8(MFRC630_REG_T3_RELOAD_LO, 0x10);   // Write default T3 reload value Lo
  write8(MFRC630_REG_T4_RELOAD_HI, 0x00);   // Write min. T4 reload value Hi
  write8(MFRC630_REG_T4_RELOAD_LO, 0x05);   // Write min. T4 reload value Lo
  write8(MFRC630_REG_T4_CONTROL, 0xf8);     // Config T4 for AutoLPCD & AutoRestart. Set AutoTrimm bit. Start T4.
  write8(MFRC630_REG_LPCD_Q_RESULT, 0x40);  // Clear LPCD result
  write8(MFRC630_REG_RCV, 0x52);            // Set Rx_ADCmode bit
  write8(MFRC630_REG_RX_ANA, 0x03);         // Raise receiver gain to maximum
  write8(MFRC630_REG_COMMAND, 0x01);        // Execute Rc663 command "Auto_T4" (Low power card detection and/or Auto trimming)

  // Flush CMD and FIFO
  write8(MFRC630_REG_COMMAND, 0x00);
  write8(MFRC630_REG_FIFO_CONTROL, 0xb0);

  // Clear Rx_ADCmode bit
  write8(MFRC630_REG_RCV, 0x12);

  // Get window values if not already calculated
  if ((LPCD_QMin == 0) || (LPCD_QMax == 0) || (LPCD_IMin == 0))
    getWindowValues();

  // Set window values
  write8(MFRC630_REG_LPCD_QMIN, LPCD_QMin);
  write8(MFRC630_REG_LPCD_QMAX, LPCD_QMax);
  write8(MFRC630_REG_LPCD_IMIN, LPCD_IMin);

  // Prepare LPCD command; power down time 10 ms, cmd time 150 us
  write8(MFRC630_REG_T3_RELOAD_HI, 0x07);
  write8(MFRC630_REG_T3_RELOAD_LO, 0xf2);
  write8(MFRC630_REG_T4_RELOAD_HI, 0x00);
  write8(MFRC630_REG_T4_RELOAD_LO, 0x13);

  // Configure T4 for AutoLPCD and AutoRestart/Autowakeup. Use 2Khz LFO, Start T4
  write8(MFRC630_REG_T4_CONTROL, 0xdf);

  // Clear LPCD result
  write8(MFRC630_REG_LPCD_Q_RESULT, 0x40);

  // Set Rx_ADCmode bit
  write8(MFRC630_REG_RCV, 0x52);

  // Raise receiver gain to max
  write8(MFRC630_REG_RX_ANA, 0x03);

  // Wait until T4 is started
  while(read8(MFRC630_REG_T4_CONTROL) != 0x9f) {};

 // Flush cmd and FIFO. Clear all IRQ flags.
  writeCommand(MFRC630_CMD_IDLE);
  write8(MFRC630_REG_FIFO_CONTROL, 0xb0);
  write8(MFRC630_REG_IRQ0, 0x7f);
  write8(MFRC630_REG_IRQ1, 0x7f);

  // Enable IRQ sources: Idle and LPCD
  write8(MFRC630_REG_IRQ0EN, 0x10);
  write8(MFRC630_REG_IRQ1EN, 0x60);

  // Start RC663 cmd "Low power card detection". Enter PowerDown mode.
  writeCommand(MFRC630_CMD_LPCD);

}

uint16_t iso14443aRequest()
{
  return iso14443aCommand(ISO14443_CMD_REQA);
}

void clearFIFO()
{
  uint8_t ctrl = read8(MFRC630_REG_FIFO_CONTROL);
  write8(MFRC630_REG_FIFO_CONTROL, ctrl | (1 << 4));
}

int16_t readFIFOLen()
{
  /* Give FIFO a chance to fill up. */
  /* TODO: Why do we need a delay between reads?!? */
  halCommonDelayMilliseconds(10);

  /* Read the MFRC630_REG_FIFO_LENGTH register */
  /* In 512 byte mode, the upper two bits are stored in FIFO_CONTROL */
  uint8_t hi = read8(MFRC630_REG_FIFO_CONTROL);
  uint8_t lo = read8(MFRC630_REG_FIFO_LENGTH);

  /* Determine len based on FIFO size (255 byte or 512 byte mode) */
  int16_t l = (hi & 0x80) ? lo : (((hi & 0x3) << 8) | lo);

  emberAfCorePrintln("FIFO contains %d byte(s)", l);
  return l;

}

int16_t readFIFO(uint16_t len, uint8_t *buffer)
{
  int16_t ctr = 0;

  /* Check for 512 byte overflow */
  if (len > 512) {
    return -1;
  }

  /* Read len bytes from the FIFO */
  for (uint16_t i=0; i<len; i++) {
    buffer[i] = read8(MFRC630_REG_FIFO_DATA);
    ctr++;
  }

  return ctr;

}

int16_t writeFIFO(uint16_t len, uint8_t *buffer)
{
  int counter = 0;

  /* Check for 512 byte overflow */
  if (len > 512) {
    return -1;
  }

  /* Write len bytes to the FIFO */
  for (uint16_t i = 0; i < len; i++) {
    write8(MFRC630_REG_FIFO_DATA, buffer[i]);
    counter++;
  }

  return counter;

}

uint16_t iso14443aCommand(uint8_t cmd)
{
  uint16_t atqa = 0; /* Answer to request (2 bytes). */
  uint8_t irqval = 0;

  /* Cancel any current command */
  writeCommand(MFRC630_CMD_IDLE);

  /* Flush the FIFO */
  clearFIFO();

  /*
   * Define the number of bits from the last byte should be sent. 000 means
   * that all bits of the last data byte are sent, 1..7 causes the specified
   * number of bits to be sent. Also set the DataEn bit to enable data xfer.
   */
  write8(MFRC630_REG_TX_DATA_NUM, 0x07 | (1 << 3));

  /* Disable CRC. */
  write8(MFRC630_REG_TX_CRC_PRESET, 0x18);
  write8(MFRC630_REG_RX_CRC_CON, 0x18);

  /* Clear the receiver control register. */
  write8(MFRC630_REG_RX_BIT_CTRL, 0);

  /* Clear the interrupts. */
  write8(MFRC630_REG_IRQ0, 0b01111111);
  write8(MFRC630_REG_IRQ1, 0b00111111);
  /* Allow the receiver and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ0EN, MFRC630IRQ0_RXIRQ | MFRC630IRQ0_ERRIRQ);
  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (5ms max). */
  write8(MFRC630_REG_T0_CONTROL, 0b10001);
  write8(MFRC630_REG_T0_RELOAD_HI, 1100 >> 8);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 1100 >> 8);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Send the ISO14443 command. */
  writeParamCommand(MFRC630_CMD_TRANSCEIVE, 1, &cmd);

  /* Wait here until we're done reading, get an error, or timeout. */
  /* TODO: Update to use timeout parameter! */
  while (!(irqval & MFRC630IRQ1_TIMER0IRQ)) {
    irqval = read8(MFRC630_REG_IRQ1);
    /* Check for a global interrupt, which can only be ERR or RX. */
    if (irqval & MFRC630IRQ1_GLOBALIRQ) {
      break;
    }
  }

  /* Cancel the current command (in case we timed out or error occurred). */
  writeCommand(MFRC630_CMD_IDLE);

  /* Check the RX IRQ, and exit appropriately if it has fired (error). */
  irqval = read8(MFRC630_REG_IRQ0);
  if ((!(irqval & MFRC630IRQ0_RXIRQ) || (irqval & MFRC630IRQ0_ERRIRQ))) {
    /* Display the error message if ERROR IRQ is set. */
    if (irqval && MFRC630IRQ0_ERRIRQ) {
      uint8_t error = read8(MFRC630_REG_ERROR);
      /* Only display the error if it isn't a timeout. */
      if (error) {
        printError(error);
      }
    }

    return 0;

  }

  /* Read the response */
  uint16_t rxlen = readFIFOLen();
  if (rxlen == 2) {
    /*
     * If we have 2 bytes for the response, it's the ATQA.
     *
     * See ISO14443-3 6.3.2 for help in interpreting the ATQA value.
     *
     * "After a REQA Command is transmitted by the PCD, all
     * PICCs in the IDLE State shall respond synchronously with ATQA."
     *
     * 0x44 = 4 bit frame anticollision
     *        UID size = double
     */
    readFIFO(rxlen, (uint8_t *)&atqa);

    emberAfCorePrintln("Received response (ATQA): 0x%2x", atqa);
    return atqa;
  }

  return 0;

}

uint8_t iso14443aSelect(uint8_t *uid, uint8_t *sak)
{
  /* Cancel any current command */
  writeCommand(MFRC630_CMD_IDLE);

  /* Flush the FIFO */
  clearFIFO();

  /* Allow the receiver and Error IRQs to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ0EN, MFRC630IRQ0_RXIRQ | MFRC630IRQ0_ERRIRQ);

  /* Allow Timer0 IRQ to be propagated to the GlobalIRQ. */
  write8(MFRC630_REG_IRQ1EN, MFRC630IRQ1_TIMER0IRQ);

  /* Configure the frame wait timeout using T0 (5ms max). */
  /* 1 'tick' 4.72us, so 1100 = 5.2ms */
  write8(MFRC630_REG_T0_CONTROL, 0b10001);
  write8(MFRC630_REG_T0_RELOAD_HI, 1100 >> 8);
  write8(MFRC630_REG_TO_RELOAD_LO, 0xFF);
  write8(MFRC630_REG_T0_COUNTER_VAL_HI, 1100 >> 8);
  write8(MFRC630_REG_T0_COUNTER_VAL_LO, 0xFF);

  /* Set the cascade level (collision detection loop) */
  for (uint8_t cascadelvl = 1; cascadelvl <= 3; cascadelvl++) {
    uint8_t cmd;
    uint8_t kbits = 0;                        /* Bits known in UID so far. */
    uint8_t send_req[7] = {0};                /* TX buffer */
    uint8_t *uid_this_level = &(send_req[2]); /* UID pointer */
    uint8_t message_length;

    switch (cascadelvl) {
    case 1:
      cmd = ISO14443_CAS_LEVEL_1;
      break;
    case 2:
      cmd = ISO14443_CAS_LEVEL_2;
      break;
    case 3:
      cmd = ISO14443_CAS_LEVEL_3;
      break;
    }

    /* Disable CRC. */
    write8(MFRC630_REG_TX_CRC_PRESET, 0x18);
    write8(MFRC630_REG_RX_CRC_CON, 0x18);

    /* As per ISO14443-3, limit collision checks to 32 attempts. */
    uint8_t cnum;
    for (cnum = 0; cnum < 32; cnum++) {
      /* Clear the interrupts. */
      write8(MFRC630_REG_IRQ0, 0b01111111);
      write8(MFRC630_REG_IRQ1, 0b00111111);

      /* Send the current collision level command */
      send_req[0] = cmd;
      send_req[1] = 0x20 + kbits;

      /* Limit MFRC630_REG_TX_DATA_NUM to the correct number of bits. */
      write8(MFRC630_REG_TX_DATA_NUM, (kbits % 8) | (1 << 3));

      // ValuesAfterColl: If cleared, every received bit after a collision is
      // replaced by a zero. This function is needed for ISO/IEC14443
      // anticollision (0<<7). We want to shift the bits with RxAlign
      uint8_t rxalign = kbits % 8;
      write8(MFRC630_REG_RX_BIT_CTRL, (0 << 7) | (rxalign << 4));

      /* Determine the message length */
      if ((kbits % 8) == 0) {
        message_length = ((kbits / 8)) + 2;
      } else {
        message_length = ((kbits / 8) + 1) + 2;
      }

      /* Send the command. */
      writeParamCommand(MFRC630_CMD_TRANSCEIVE, message_length, send_req);

      /* Wait until the command execution is complete. */
      uint8_t irq1_value = 0;
      while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
        irq1_value = read8(MFRC630_REG_IRQ1);
        /* Check for a global interrupt, which can only be ERR or RX. */
        if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
          break;
        }
      }

      /* Cancel any current command */
      writeCommand(MFRC630_CMD_IDLE);

      /* Parse results */
      uint8_t irq0_value = read8(MFRC630_REG_IRQ0);
      uint8_t error = read8(MFRC630_REG_ERROR);
      uint8_t coll = read8(MFRC630_REG_RX_COLL);
      uint8_t coll_p = 0;

      /* Check if an error occurred */
      if (irq0_value & MFRC630IRQ0_ERRIRQ) {
        /* Display the error code in human-readable format. */
        printError(error);
        if (error & MFRC630_ERROR_COLLDET) {
          /* Collision error, check if the collision position is valid */
          if (coll & (1 << 7)) {
            /* Valid, so check the collision position (bottom 7 bits). */
            coll_p = coll & (~(1 << 7));
            uint8_t choice_pos = kbits + coll_p;
            uint8_t selection = (uid[((choice_pos + (cascadelvl - 1) * 3) / 8)] >> ((choice_pos) % 8)) & 1;
            uid_this_level[((choice_pos) / 8)] |= selection << ((choice_pos) % 8);
            kbits++;
          } else {
            coll_p = 0x20 - kbits;
          } /* End: if (coll & (1 << 7)) */
        } else {
          coll_p = 0x20 - kbits;
        } /* End: if (error & MFRC630_ERROR_COLLDET) */
      } else if (irq0_value & MFRC630IRQ0_RXIRQ) {
        /* We have data and no collision, all is well in the world! */
        coll_p = 0x20 - kbits;
      } else {
        return 0;
      }

      /* Read the UID so far */
      uint16_t rxlen = readFIFOLen();
      uint8_t buf[5]; /* UID = 4 bytes + BCC */
      readFIFO(rxlen < 5 ? rxlen : 5, buf);

      /*
       * Move current buffer contents into the UID placeholder, OR'ing the
       * results so that we don't lose the bit we set if you have a collision.
       */
      uint8_t rbx;
      for (rbx = 0; (rbx < rxlen); rbx++) {
        uid_this_level[(kbits / 8) + rbx] |= buf[rbx];
      }
      kbits += coll_p;

      if ((kbits >= 32)) {
        break; /* Exit the collision loop */
      }
    } /* End: for (cnum = 0; cnum < 32; cnum++) */

    /* Check if the BCC matches ... */
    uint8_t bcc_val = uid_this_level[4];
    uint8_t bcc_calc = uid_this_level[0] ^ uid_this_level[1] ^ uid_this_level[2] ^ uid_this_level[3];
    if (bcc_val != bcc_calc) {
        emberAfCorePrintln("ERROR: BCC mismatch!");
      return 0;
    }

    /* Clear the interrupts. */
    write8(MFRC630_REG_IRQ0, 0b01111111);
    write8(MFRC630_REG_IRQ1, 0b00111111);

    send_req[0] = cmd;
    send_req[1] = 0x70;
    send_req[6] = bcc_calc;
    message_length = 7;

    /* Re-enable CRCs. */
    write8(MFRC630_REG_TX_CRC_PRESET, 0x18 | 1);
    write8(MFRC630_REG_RX_CRC_CON, 0x18 | 1);

    /* Reset the TX and RX registers (disable alignment, transmit full bytes) */
    write8(MFRC630_REG_TX_DATA_NUM, (kbits % 8) | (1 << 3));
    uint8_t rxalign = 0;
    write8(MFRC630_REG_RX_BIT_CTRL, (0 << 7) | (rxalign << 4));

    /* Send the command. */
    writeParamCommand(MFRC630_CMD_TRANSCEIVE, message_length, send_req);

    /* Wait until the command execution is complete. */
    uint8_t irq1_value = 0;
    while (!(irq1_value & MFRC630IRQ1_TIMER0IRQ)) {
      irq1_value = read8(MFRC630_REG_IRQ1);
      /* Check for a global interrupt, which can only be ERR or RX. */
      if (irq1_value & MFRC630IRQ1_GLOBALIRQ) {
        break;
      }
    }
    writeCommand(MFRC630_CMD_IDLE);

    /* Check the source of exiting the loop. */
    uint8_t irq0_value = read8(MFRC630_REG_IRQ0);
    /* Check the ERROR IRQ */
    if (irq0_value & MFRC630IRQ0_ERRIRQ) {
      /* Check what kind of error. */
      uint8_t error = read8(MFRC630_REG_ERROR);
      if (error & MFRC630_ERROR_COLLDET) {
        /* Collision detection. */
          emberAfCorePrintln("ERROR: Collision detected");
        return 0;
      }
    }

    /* Read SAK answer from fifo. */
    uint8_t sak_len = readFIFOLen();
    if (sak_len != 1) {
        emberAfCorePrintln("ERROR: NO SAK in response!");
      return 0;
    }
    uint8_t sak_value;
    readFIFO(sak_len, &sak_value);

    /* Check if there is more data to read. */
    if (sak_value & (1 << 2)) {
      /* UID not yet complete, continue to next cascade. */
      uint8_t UIDn;
      for (UIDn = 0; UIDn < 3; UIDn++) {
        // uid_this_level[UIDn] = uid_this_level[UIDn + 1];
        uid[(cascadelvl - 1) * 3 + UIDn] = uid_this_level[UIDn + 1];
      }
    } else {
      /* Done! */
      /* Add current bytes at this level to the UID. */
      uint8_t UIDn;
      for (UIDn = 0; UIDn < 4; UIDn++) {
        uid[(cascadelvl - 1) * 3 + UIDn] = uid_this_level[UIDn];
      }

      /* Finally, return the length of the UID that's now at the uid pointer. */
      return cascadelvl * 3 + 1;
    }
  } /* End: for (cascadelvl = 1; cascadelvl <= 3; cascadelvl++) */

  /* Return 0 for UUID length if nothing was found. */
  return 0;

}

void printError(uint8_t err)
{
  emberAfCorePrint("MFRC630_ERROR: ");
  switch (err) {
  case MFRC630_ERROR_INTEG:
    emberAfCorePrintln("Data integrity!");
    break;
  case MFRC630_ERROR_PROT:
    emberAfCorePrintln("Protocol error!");
    break;
  case MFRC630_ERROR_COLLDET:
    emberAfCorePrintln("Collision detected!");
    break;
  case MFRC630_ERROR_NODATA:
    emberAfCorePrintln("No data!");
    break;
  case MFRC630_ERROR_MINFRAME:
    emberAfCorePrintln("Frame data too small!");
    break;
  case MFRC630_ERROR_FIFOOVL:
    emberAfCorePrintln("FIFO full!");
    break;
  case MFRC630_ERROR_FIFOWR:
    emberAfCorePrintln("Couldn't write to FIFO!");
    break;
  case MFRC630_ERROR_EEPROM:
    emberAfCorePrintln("EEPROM access!");
    break;
  default:
    emberAfCorePrintln("Unhandled error code: 0x%x", err);
    break;
  }
}

bool readRfidTag(rfid_tag_t* rfid_tag)
{
  uint16_t atqa = iso14443aRequest();

  if (atqa) {
    uint8_t uid[10] = { 0 };
    uint8_t len;
    uint8_t sak;

    len = iso14443aSelect(uid, &sak);

    if (len >= 4 && len <= 10) {
      memcpy(rfid_tag->rfid, uid, len);
      rfid_tag->size = len;
      rfid_tag->sak = sak;
      return true;
    }

  }

  return false;

}

uint8_t readEeprom(const uint8_t page, const uint8_t offset, const uint8_t length, uint8_t* buffer)
{
  // Check values
  if ((page > 127) || (offset + length > 64))
    return 0;

  // Get params
  uint16_t address = page * 64 + offset;
  uint8_t parameters[3] = {(uint8_t)(address >> 8), (uint8_t)(address & 0xff), length};

  // Send command
  clearFIFO();
  halCommonDelayMilliseconds(10);
  writeFIFO(3, parameters);
  halCommonDelayMilliseconds(10);
  write8(MFRC630_REG_COMMAND, MFRC630_CMD_READE2);
  halCommonDelayMilliseconds(10);

  // Check length
  uint16_t res = readFIFOLen();

  if (res != length) {
    uint8_t error = read8(MFRC630_REG_ERROR);
    if (error > 0)
      printError(error);
    else
      emberAfCorePrintln("no error");
    return res;
  }

  // Read FIFO
  return readFIFO(length, buffer);

}
