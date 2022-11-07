/***************************************************************************//**
 * @file main.c
 * @brief main() function.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_component_catalog.h"
#include "sl_system_init.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#ifdef EMBER_TEST
#define main nodeMain
#endif

#include "app/framework/include/af.h"

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_wdog.h"

#include "gpiointerrupt.h"

#include "rfid.h"

// 5V control
#define ENABLE_5V_PORT          gpioPortD
#define ENABLE_5V_PIN           3

// I2C
#define I2C_PORT                gpioPortC
#define SCL_PIN                 3
#define SDA_PIN                 4
#define RFID_INT_PORT           gpioPortB
#define RFID_INT_PIN            0
#define RFID_RESET_PORT         gpioPortD
#define RFID_RESET_PIN          4
#define RFID_IRQ_NO             1
#define I2C_FREQUENCY           200000              // Hz

uint8_t rfidAddress = 0x28;

bool handlingTag = false;
bool okToSleep = true;
bool rfidIrq = false;

/** @brief Handle RFID interrupt
 *  @param pin GPIO pin
 *  @note Triggered by LPCD (low power card detect; i.e. a card has been detected)
 *  @note Actual handling is done in app_process_action
 */
void rfid_irq_handler(uint8_t intNo)
{
  GPIO_ExtIntConfig(RFID_INT_PORT, RFID_INT_PIN, RFID_IRQ_NO, false, false, false);
  emberAfCorePrintln("rfid irq: intNo = %d", intNo);
  rfidIrq = (intNo == RFID_IRQ_NO);
}

/** @brief Initialize GPIO pins
 */
static void initGpio(void)
{
  // Disable SWD to allow cli on PA1 and PA2
  GPIO_DbgSWDClkEnable(false);
  GPIO_DbgSWDIOEnable(false);

  // Configure 5V control
  GPIO_PinModeSet(ENABLE_5V_PORT, ENABLE_5V_PIN, gpioModePushPull, 0);

  // Enable GPIO interrupts
  GPIOINT_Init();

  // I2C (RFID reader)
  GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
                             | (I2C_PORT << _GPIO_I2C_SDAROUTE_PORT_SHIFT
                             | (SDA_PIN << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
                             | (I2C_PORT << _GPIO_I2C_SCLROUTE_PORT_SHIFT
                             | (SCL_PIN << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;
  GPIO_PinModeSet(I2C_PORT, SDA_PIN, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(I2C_PORT, SCL_PIN, gpioModeWiredAnd, 1);

  // Configure RFID interrupt and reset pins
  GPIO_PinModeSet(RFID_RESET_PORT, RFID_RESET_PIN, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(RFID_INT_PORT, RFID_INT_PIN, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(RFID_INT_PORT, RFID_INT_PIN, RFID_IRQ_NO, true, false, true);
  GPIOINT_CallbackRegister(RFID_IRQ_NO, rfid_irq_handler);

}

/** @brief Initialize I2C (used for RFID reader)
 *  @note GPIO routing and pin configurations are done in initGpio
 */
static void initI2C(void)
{
  // Enable clock
  CMU_ClockEnable(cmuClock_I2C0, true);

  uint8_t i2cClockHLRStandard = 0;

  // Using default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

  // Set frequency
  i2cInit.freq = I2C_FREQUENCY;

  // Initialize
  I2C_Init(I2C0, &i2cInit);

}

/** @brief Enable or disable RFID
 *  @param enable If true, RFID is enabled, else RFID is disabled
 */
static void enableRfid(bool enable)
{
  if (enable) {
    GPIO_PinOutClear(RFID_RESET_PORT, RFID_RESET_PIN);
    halCommonDelayMilliseconds(10);
  }
  else
    GPIO_PinOutSet(RFID_RESET_PORT, RFID_RESET_PIN);

  emberAfCorePrintln("%s rfid", enable ? "enable" : "disable");

}

/** @brief RFID hard reset
 */
static void rfidHardReset(void)
{
  enableRfid(true);
  halCommonDelayMilliseconds(5);
  enableRfid(false);
  halCommonDelayMilliseconds(5);
  enableRfid(true);
  halCommonDelayMilliseconds(10);
}

static char* productString(const uint8_t ver)
{
  switch (ver) {
    case 0x01:
      return "CLRC663";
    case 0xc0:
      return "MFRC631";
    case 0x80:
      return "MFRC630";
    case 0x20:
      return "SLRC610";
    default:
      return "unknown";
  }
}

/** @brief Print version of RFIC chip
 */
static void printRfidVersion(void)
{
  uint8_t version, subVersion = 0;

  version = read8(MFRC630_REG_VERSION);

  if ((version >> 4) == 0x01) {
    subVersion = version & 0b1111;
  }

  emberAfCorePrintln("version = 0x%x, subversion = 0x%x", version, subVersion);

  uint8_t len;
  const uint8_t bufLen = 8;
  uint8_t buf[bufLen];

  // Read_eeprom
  emberAfCorePrintln("read eeprom");
  len = readEeprom(0, 0, bufLen, buf);

  if (len == bufLen) {
    uint8_t productId = buf[1];
    uint8_t productVersion = buf[3];
    emberAfCorePrintln("product id = 0x%x (%s), product version = %d", productId, productString(productId), productVersion);
  }
  else {
    emberAfCorePrintln("failed to get product information from EEPROM; len = %d", len);
  }

  emberAfCorePrintln("version of silicon is %s", subVersion == 0x08 ? "CLRC66301 or CLRC66302" : "CLRC66303");

}

static void readTag(rfid_tag_t* rfid_tag, const uint8_t maxAttempts)
{
  // Read interrupt registers
  uint8_t regVal0 = read8(MFRC630_REG_IRQ0);
  uint8_t regVal1 = read8(MFRC630_REG_IRQ1);
  emberAfCorePrintln("Before read: MFRC630_REG_IRQ0 = 0x%x, MFRC630_REG_IRQ1 = 0x%x", regVal0, regVal1);

  // Read tag
  for (int i = 0; i < maxAttempts; i++) {
    if (readRfidTag(rfid_tag)) {
      emberAfCorePrintln("===================================");
      emberAfCorePrintln("read tag, size = %d, sak = %d", rfid_tag->size, rfid_tag->sak);
      emberAfCorePrint("tag:");
      for (int i = 0; i < rfid_tag->size; i++)
        emberAfCorePrint(" %x", rfid_tag->rfid[i]);
      emberAfCorePrintln("");
      emberAfCorePrintln("===================================");
      break;
    }
    else {
      emberAfCorePrintln("failed to read tag");
      WDOGn_Feed(DEFAULT_WDOG);
      halCommonDelayMilliseconds(10);
    }
  }

  // Read interrupt registers
  regVal0 = read8(MFRC630_REG_IRQ0);
  regVal1 = read8(MFRC630_REG_IRQ1);
  emberAfCorePrintln("After read: MFRC630_REG_IRQ0 = 0x%x, MFRC630_REG_IRQ1 = 0x%x", regVal0, regVal1);

}

static void handleTag(void)
{
  rfid_tag_t rfid_tag;

  // Initialize RFID
  rfidInit();

  // Try to read tag; check result
  readTag(&rfid_tag, 3);

  halCommonDelayMilliseconds(100);

  // Reset
  rfidLpcdInit();
  handlingTag = false;
  rfidIrq = false;
  GPIO_ExtIntConfig(RFID_INT_PORT, RFID_INT_PIN, RFID_IRQ_NO, true, false, true);
  okToSleep = true;

}

void app_init(void)
{
  initGpio();
  initI2C();

  // Print reset cause
  emberAfCorePrintln("Reset info: 0x%x (%p)", halGetResetInfo(), halGetResetString());
  emberAfCorePrintln("Extended Reset info: 0x%2X (%p)", halGetExtendedResetInfo(), halGetExtendedResetString());

  // Show FW version
  emberAfCorePrintln("fw version = %s", "1.0");

  // RFID hard reset
  rfidHardReset();

  // Print RFID version
  printRfidVersion();

  // Enter LPCD mode
  rfidLpcdInit();

}

void app_process_action(void)
{
  if (rfidIrq) {

    okToSleep = false;

    printQIValues();

    // Get Irq1 status
    uint8_t regVal = read8(MFRC630_REG_IRQ1);

    emberAfCorePrintln("MFRC630_REG_IRQ1 == 0x%x", regVal);

    // Check if LPCD irq is triggered
    if ((regVal & 0x20) != 0) {

      emberAfCorePrintln("lpcd irq");

      emberAfCorePrintln("Flush any running command and FIFO");
      write8(MFRC630_REG_COMMAND, 0x00);
      write8(MFRC630_REG_IRQ0EN, 0x00);
      write8(MFRC630_REG_IRQ1EN, 0x00);

      write8(MFRC630_REG_FIFO_CONTROL, 0xb0);

      // Read error status register
      uint8_t errorStatus = read8(MFRC630_REG_ERROR);
      emberAfCorePrintln("error status register = 0x%x", errorStatus);

      // Clear Rx_ADCmode bit
      write8(MFRC630_REG_RCV, 0x12);

      // Stop Timer4
      write8(MFRC630_REG_T4_CONTROL, 0x5f);

      if (!handlingTag) {
        handlingTag = true;
        handleTag();
      }

    }

    else {
      rfidLpcdInit();
      rfidIrq = false;
      GPIO_ExtIntConfig(RFID_INT_PORT, RFID_INT_PIN, RFID_IRQ_NO, true, false, true);
      okToSleep = true;
    }

  }

}

bool app_is_ok_to_sleep(void)
{
  return okToSleep;
}

int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  app_init();

#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    app_process_action();

    // Let the CPU go to sleep if the system allow it.
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    sl_power_manager_sleep();
#endif // SL_CATALOG_POWER_MANAGER_PRESENT
  }
#endif // SL_CATALOG_KERNEL_PRESENT

  return 0;
}
