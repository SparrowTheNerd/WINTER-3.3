#include "MS5607.h"

/**
 * @brief  Initializes the MS5607
 * @param  *dev: Pointer to the MS5607 structure
 * @param  *hi2c: Pointer to the I2C handle
 * @param  OSR: Oversample rate (0-4 -> 256-4096)
 * @note   OSR param can be 0 thru 4; see MS5607 datasheet
 * @retval None
 */
MS5607::MS5607(I2C_HandleTypeDef *hi2c, uint8_t OSR) {
    this->hi2c = hi2c;
    this->OSR = OSR;
    this->available = 0;
    this->temp = 0.0f;
    this->pres = 0;
    this->alt = 0.0f;
}

/**
 * @brief  Initializes the MS5607
 * @param  *dev: Pointer to the MS5607 structure
 * @param  *hi2c: Pointer to the I2C handle
 * @retval HAL Status
 */
HAL_StatusTypeDef MS5607::Init() {
    // Reset the device
    uint8_t resetCmd = MS5607_REG_RESET;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, MS5607_ADDR, &resetCmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(10); // Wait for reset to complete

    // Read calibration coefficients from PROM
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t regAddr = MS5607_REG_PROM_COEF + (i * 2);
        uint8_t data[2];
        status = ReadRegs(regAddr, data, 2);
        if (status != HAL_OK) {
            return status;
        }
        raw.C[i] = (data[0] << 8) | data[1];
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
HAL_StatusTypeDef MS5607::ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(hi2c, MS5607_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

uint8_t baroStep = 0;
/**
 * @brief  Reads data from the MS5607
 * @param  *dev: Pointer to the MS5607 structure
 * @retval HAL Status
 */
HAL_StatusTypeDef MS5607::GetData() {
    // Read digital pressure value D1
    HAL_StatusTypeDef status;
    switch(baroStep) {
        case 0:
            cmd = MS5607_REG_CONVERT_D1 + (OSR * 2);
            status = HAL_I2C_Master_Transmit(hi2c, MS5607_ADDR, &cmd, 1, HAL_MAX_DELAY);   // Start conversion for pressure
            if(status != HAL_OK) {
                return status;
            }
            baroStep = 1;
            break;
        case 1:
            cmd = MS5607_REG_ADC_READ;
            uint8_t buf[3];
            status = HAL_I2C_Mem_Read(hi2c, MS5607_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
            if(status != HAL_OK) {
                return status;
            }

            raw.D1 = (buf[0] << 16) | (buf[1] << 8) | buf[2]; // Combine the bytes into a 24-bit value
            raw.D1 = raw.D1 & (uint32_t)0x00FFFFFF; // Clear the last 4 bits

            cmd = MS5607_REG_CONVERT_D2 + (OSR * 2);
            status = HAL_I2C_Master_Transmit(hi2c, MS5607_ADDR, &cmd, 1, HAL_MAX_DELAY);   // Start conversion for temperature
            if(status != HAL_OK) {
                return status;
            }

            baroStep = 2;
            break;
        case 2:
            cmd = MS5607_REG_ADC_READ;
            status = HAL_I2C_Mem_Read(hi2c, MS5607_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
            if(status != HAL_OK) {
                return status;
            }

            raw.D2 = (buf[0] << 16) | (buf[1] << 8) | buf[2]; // Combine the bytes into a 24-bit value
            raw.D2 = raw.D2 & (uint32_t)0x00FFFFFF; // Clear the last 4 bits

            baroStep = 0; // Reset step for next read
            Convert();
            break;
    }
    return status;
}

/**
 * @brief  Converts raw data to pressure and temperature
 * @param  *dev: Pointer to the MS5607 structure
 * @retval None
 */
void MS5607::Convert() {
    raw.dT = raw.D2 - ((uint32_t)raw.C[4] << 8);
    raw.TEMP = 2000 + ((int32_t)raw.dT * (int32_t)raw.C[5] >> 23);

    raw.OFF = ((int64_t)raw.C[1] << 17) + ((int64_t)raw.dT * (int64_t)raw.C[3] >> 6);
    raw.SENS = ((int64_t)raw.C[0] << 16) + ((int64_t)raw.dT * (int64_t)raw.C[2] >> 7);
    raw.P = (int32_t)(((raw.D1 * raw.SENS >> 21) - raw.OFF) >> 15);


    temp = (float)raw.TEMP / 100.0f; // Convert temperature to Celsius
    pres = raw.P; // Pressure in Pa
    alt = (float)((1.f - pow((pres) / (double)101325, (double)0.190284)) * (double)145366.45); // Altitude in meters
}