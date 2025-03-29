#include "abstract.h"
#include "usbd_cdc_if.h"
#include "i2c.h"

/**
  * @brief  Prints a string to the USB serial port
  * @param  txBuf[]: Character array to transmit
  * @retval HAL Status
  */
uint8_t SerialPrint(uint8_t txBuf[]) {
    uint16_t bufLen = strlen((char*)txBuf);
    return CDC_Transmit_HS(txBuf, bufLen);
}

/**
  * @brief  Prints a string and newline to the USB serial port
  * @param  txBuf[]: Character array to transmit
  * @retval HAL Status
  */
uint8_t SerialPrintln(uint8_t txBuf[]) {
    uint16_t bufLen = strlen((char*)txBuf);
    char fullBuf[bufLen + 3];
    strcpy(fullBuf, (char *) txBuf);
    strcat(fullBuf,"\r\n");
    return CDC_Transmit_HS((uint8_t*)fullBuf, strlen(fullBuf));
}

/**
  * @brief  Reads from an I2C device
  * @param  addr: I2C address of the device
  * @param  reg: Register to read from
  * @param  data: Pointer to the data buffer
  * @param  len: Number of bytes to read
  * @retval HAL Status
  */
uint8_t i2cRead(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Read(&hi2c3, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 1000);
}

uint8_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Write(&hi2c3, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 1000);
}