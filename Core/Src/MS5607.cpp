#include "MS5607.h"

/**
 * @brief  Initializes the MS5607
 * @param  *dev: Pointer to the MS5607 structure
 * @param  *hi2c: Pointer to the I2C handle
 * @param  OSR: Oversample rate (0-4 -> 256-4096)
 * @note   OSR param can be 0 thru 4; see MS5607 datasheet
 * @retval HAL Status
 */
HAL_StatusTypeDef MS5607_Init(MS5607 *dev, I2C_HandleTypeDef *hi2c, uint8_t OSR) {
    dev->hi2c = hi2c;
    dev->OSR = OSR;
    dev->temp = 0.0f;
    dev->pres = 0.0f;
    dev->alt = 0.0f;

    // Reset the device
    uint8_t resetCmd = MS5607_REG_RESET;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, MS5607_ADDR, &resetCmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(10); // Wait for reset to complete

    // Read calibration coefficients from PROM
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t regAddr = MS5607_REG_PROM_COEF + (i * 2);
        uint8_t data[2];
        status = MS5607_ReadRegs(dev, regAddr, data, 2);
        if (status != HAL_OK) {
            return status;
        }
        dev->raw.C[i] = (data[0] << 8) | data[1];
    }

    return HAL_OK;
}

/**
  * @brief  Reads Register(s) from the MS5607
  * @param  *dev: Pointer to the MS5607 structure
  * @param  reg: Register to read from
  * @param  *data: Pointer to the data buffer
  * @param  len: Number of bytes to read
  * @retval HAL Status
  */
HAL_StatusTypeDef MS5607_ReadRegs(MS5607 *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(dev->hi2c, MS5607_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

uint8_t baroStep = 0;
/**
 * @brief  Reads data from the MS5607
 * @param  *dev: Pointer to the MS5607 structure
 * @retval HAL Status
 */
HAL_StatusTypeDef MS5607_GetData(MS5607 *dev) {
    // Read digital pressure value D1
    HAL_StatusTypeDef status;
    uint8_t cmd;
    switch(baroStep) {
        case 0:
            cmd = MS5607_REG_CONVERT_D1 + (dev->OSR * 2);
            status = HAL_I2C_Master_Transmit(dev->hi2c, MS5607_ADDR, &cmd, 1, HAL_MAX_DELAY);   // Start conversion for pressure
            if(status != HAL_OK) {
                return status;
            }
            baroStep = 1;
            break;
        case 1:
            cmd = MS5607_REG_ADC_READ;
            uint8_t buf[3];
            status = HAL_I2C_Mem_Read(dev->hi2c, MS5607_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
            if(status != HAL_OK) {
                return status;
            }

            dev->raw.D1 = (buf[0] << 16) | (buf[1] << 8) | buf[2]; // Combine the bytes into a 24-bit value
            dev->raw.D1 = dev->raw.D1 & (uint32_t)0x00FFFFFF; // Clear the last 4 bits

            cmd = MS5607_REG_CONVERT_D2 + (dev->OSR * 2);
            status = HAL_I2C_Master_Transmit(dev->hi2c, MS5607_ADDR, &cmd, 1, HAL_MAX_DELAY);   // Start conversion for temperature
            if(status != HAL_OK) {
                return status;
            }

            baroStep = 2;
            break;
        case 2:
            cmd = MS5607_REG_ADC_READ;
            status = HAL_I2C_Mem_Read(dev->hi2c, MS5607_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
            if(status != HAL_OK) {
                return status;
            }

            dev->raw.D2 = (buf[0] << 16) | (buf[1] << 8) | buf[2]; // Combine the bytes into a 24-bit value
            dev->raw.D2 = dev->raw.D2 & (uint32_t)0x00FFFFFF; // Clear the last 4 bits

            baroStep = 0; // Reset step for next read
            MS5607_Convert(dev);
            break;
    }
    return status;
}

/**
 * @brief  Converts raw data to pressure and temperature
 * @param  *dev: Pointer to the MS5607 structure
 * @retval None
 */
void MS5607_Convert(MS5607 *dev) {
    dev->raw.dT = dev->raw.D2 - ((uint32_t)dev->raw.C[4] << 8);
    dev->raw.TEMP = 2000 + ((int32_t)dev->raw.dT * (int32_t)dev->raw.C[5] >> 23);

    dev->raw.OFF = ((int64_t)dev->raw.C[1] << 17) + ((int64_t)dev->raw.dT * (int64_t)dev->raw.C[3] >> 6);
    dev->raw.SENS = ((int64_t)dev->raw.C[0] << 16) + ((int64_t)dev->raw.dT * (int64_t)dev->raw.C[2] >> 7);
    dev->raw.P = (int32_t)(((dev->raw.D1 * dev->raw.SENS >> 21) - dev->raw.OFF) >> 15);


    dev->temp = (float)dev->raw.TEMP / 100.0f; // Convert temperature to Celsius
    dev->pres = dev->raw.P; // Pressure in Pa
    dev->alt = (float)((1.f - pow((dev->pres) / (double)101325, (double)0.190284)) * (double)145366.45); // Altitude in meters
}