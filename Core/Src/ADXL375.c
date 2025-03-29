#include "ADXL375.h"

/*
* INITIALIZATION
*/

/**
 * @brief  Initializes the ADXL375
 * @param  *adxl: Pointer to the ADXL375 structure
 * @param  *hi2c: Pointer to the I2C handle
 * @param  ODR: Output data rate
 * @param ofst[3]: [X,Y,Z] offsets (m/s^2)
 * @note   ODR param can be 0 thru 15; see ADXL375 datasheet Table 6 for details
 * @retval HAL Status
 */
HAL_StatusTypeDef ADXL375_Init(ADXL375 *adxl, I2C_HandleTypeDef *hi2c, uint8_t ODR, int8_t ofset[3]) {
    adxl->hi2c = hi2c;
    
    adxl->accel_ms2[0] = 0.0f;
    adxl->accel_ms2[1] = 0.0f;
    adxl->accel_ms2[2] = 0.0f;

    uint8_t regDat = 0;

    // Sanity check device ID
    HAL_StatusTypeDef status = ADXL375_ReadReg(adxl, ADXL375_REG_DEVID, &regDat, 1);
    if (status != HAL_OK) {
        return status;
    }
    if (regDat != ADXL375_DEVID) {
        return HAL_ERROR; // Device ID mismatch
    }

    // Set the ODR
    if(ODR > 15) { ODR = 10; } // Ensure ODR is within bounds
    regDat = ODR & 0x0F; // Mask to set ODR
    status = ADXL375_WriteReg(adxl,ADXL375_REG_BW_RATE,&regDat);
    if (status != HAL_OK) { return status; }

    // Set up offsets
    float scaleFac = 1.f/(9.80665f * 0.196f); // Offset register has scale of 0.196 g/LSB of register
    for(int i=0; i<3; i++) {
        adxl->ofst[i] = (int8_t)(ofset[i] * scaleFac);
    }

    // Power on the device
    regDat = 0x08; // Set the device to measure mode
    status = ADXL375_WriteReg(adxl, ADXL375_REG_POWER_CTL, &regDat);
    if (status != HAL_OK) { return status; }

    return HAL_OK;
}

/*
* DATA ACQUISITION
*/

/**
 * @brief  Reads acceleration data from the ADXL375
 * @param  *adxl: Pointer to the ADXL375 structure
 * @retval HAL Status
 */
HAL_StatusTypeDef ADXL375_ReadAccel(ADXL375 *adxl) {
    uint8_t data[6];
    HAL_StatusTypeDef status = ADXL375_ReadReg(adxl, ADXL375_REG_DATAX0, data, 6);
    if (status != HAL_OK) {
        return status;
    }

    // Convert the data to acceleration in m/s^2
    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);

    adxl->accel_ms2[0] = (float)x * ADXL375_CONVERT; 
    adxl->accel_ms2[1] = (float)y * ADXL375_CONVERT;
    adxl->accel_ms2[2] = (float)z * ADXL375_CONVERT;

    return HAL_OK;
}


/*
* LOW LEVEL FUNCTIONS
*/

/**
  * @brief  Reads Register(s) from the ADXL375
  * @param  *adxl: Pointer to the ADXL375 structure
  * @param  reg: Register to read from
  * @param  *data: Pointer to the data buffer
  * @param  len: Number of bytes to read
  * @retval HAL Status
  */
HAL_StatusTypeDef ADXL375_ReadReg(ADXL375 *adxl, uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Read(adxl->hi2c, ADXL375_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Write Register to the ADXL375
  * @param  *adxl: Pointer to the ADXL375 structure
  * @param  reg: Register to write to
  * @param  *data: Pointer to the data buffer
  * @retval HAL Status
  */
HAL_StatusTypeDef ADXL375_WriteReg(ADXL375 *adxl, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(adxl->hi2c, ADXL375_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}