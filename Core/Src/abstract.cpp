#include "abstract.h"
#include "usbd_cdc_if.h"
#include "i2c.h"
#include "adc.h"


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

/**
  * @brief  Writes to an I2C device
  * @param  addr: I2C address of the device
  * @param  reg: Register to write to
  * @param  data: Pointer to the data buffer
  * @param  len: Number of bytes to write
  * @retval HAL Status
  */
uint8_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Write(&hi2c3, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 1000);
}

/**
 * @brief Creates a microsecond delay using DWT cycle counter
 * @param us: Number of microseconds to delay
 */
void delay_us(uint16_t us) {
    uint32_t start = DWT->CYCCNT; // Get current cycle count
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * us; // Calculate cycles for the delay

    while ((DWT->CYCCNT - start) < cycles) {
        // Wait until the specified number of cycles has passed
    }
}

/**
  * @brief  Reads a single-ended analog value from the specified ADC channel
  * @param  channel: ADC channel to read from
  * @param  sampletime: ADC sampling time
  * @retval Analog value (0-65535, 16-bit resolution)
  */
uint32_t analogReadSE(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        // Configuration Error
        return 0;
    }

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        // Polling Error
        return 0;
    }

    uint32_t value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return value;
}