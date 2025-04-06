#include "ICM42688.h"

ICM42688::ICM42688(I2C_HandleTypeDef *hi2c, float a_ofst[3], float g_ofst[3], uint8_t g_fs, uint8_t g_odr, uint8_t a_fs, uint8_t a_odr) {
    this->hi2c = hi2c;
    this->accel_ofst[0] = a_ofst[0];
    this->accel_ofst[1] = a_ofst[1];
    this->accel_ofst[2] = a_ofst[2];
    this->gyro_ofst[0] = g_ofst[0];
    this->gyro_ofst[1] = g_ofst[1];
    this->gyro_ofst[2] = g_ofst[2];
}

HAL_StatusTypeDef ICM42688::Init() {
    // configure for i2c
    uint8_t regDat = 0x01;
    uint8_t status = WriteReg(ICM42688_REG_BANK_SEL, &regDat); // Select Bank 1
    uint8_t reg = ICM42688_REG_B1_INTF_CONFIG6;
    regDat = 0x5C;
    status |= WriteReg(reg, &regDat);
    regDat = 0x00;
    status |= WriteReg(ICM42688_REG_BANK_SEL, &regDat); // Select Bank 0
    reg = ICM42688_REG_B0_DRIVE_CONFIG;
    regDat = 0x09;
    status |= WriteReg(reg, &regDat); // Set drive config to 0x09 (default)    
    if (status != HAL_OK) {
        return (HAL_StatusTypeDef)status;
    }

    reg = ICM42688_REG_B0_WHO_AM_I;
    status = ReadRegs(reg, &regDat, 1);
    if (status != HAL_OK) {
        return (HAL_StatusTypeDef)status;
    }
    if (regDat != ICM42688_DEVID) {
        return HAL_ERROR; // Device ID mismatch
    }
    


    return HAL_OK;
}

/**
  * @brief  Reads Register(s) from the ICM42688
  * @param  *icm: Pointer to the ICM42688 structure
  * @param  reg: Register to read from
  * @param  *data: Pointer to the data buffer
  * @param  len: Number of bytes to read
  * @retval HAL Status
  */
 HAL_StatusTypeDef ICM42688::ReadRegs(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Read(hi2c, ICM42688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Write Register to the ICM42688
  * @param  *icm: Pointer to the ICM42688 structure
  * @param  reg: Register to write to
  * @param  *data: Pointer to the data buffer
  * @retval HAL Status
  */
HAL_StatusTypeDef ICM42688::WriteReg(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(hi2c, ICM42688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Write Registers to the ICM42688
  * @param  *icm: Pointer to the ICM42688 structure
  * @param  reg: Register to write to
  * @param  *data: Pointer to the data buffer
  * @param  len: Number of bytes to write
  * @retval HAL Status
  */
 HAL_StatusTypeDef ICM42688::WriteRegs(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Write(hi2c, ICM42688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Convert offsets to raw register values
  * @param  *icm: Pointer to the ICM42688 structure
  * @param  *ofst: Pointer to the raw offset array (9 bytes)
  * @retval None
  */
void ICM42688::convertOfst(int8_t ofst[9]) {
    int16_t gyro_raw[3], accel_raw[3];

    // Convert gyro offsets to raw register values
    for (int i = 0; i < 3; i++) {
        gyro_raw[i] = (int16_t)(gyro_ofst[i] * 32.f); // 32 LSB/dps
    }

    // Convert accel offsets to raw register values
    for (int i = 0; i < 3; i++) {
        accel_raw[i] = (int16_t)(accel_ofst[i] * 2000.f); // 2000 LSB/g
    }

    // Pack into register bytes (matching the register layout in sections 18.18 thru 18.26)
    ofst[0] = gyro_raw[0] & 0xFF;  // gX lower 8 bits
    ofst[1] = (gyro_raw[0] >> 8) & 0x0F; // gX upper 4 bits
    ofst[1] |= (gyro_raw[1] << 4) & 0xF0; // gY upper 4 bits
    ofst[2] = gyro_raw[1] & 0xFF;  // gY lower 8 bits
    ofst[3] = gyro_raw[2] & 0xFF;  // gZ lower 8 bits
    ofst[4] = (gyro_raw[2] >> 8) & 0x0F; // gZ upper 4 bits
    ofst[4] |= (accel_raw[0] << 4) & 0xF0; // aX upper 4 bits
    ofst[5] = accel_raw[0] & 0xFF;  // aX lower 8 bits
    ofst[6] = accel_raw[1] & 0xFF;  // aY lower 8 bits
    ofst[7] = (accel_raw[1] >> 8) & 0x0F; // aY upper 4 bits
    ofst[7] |= (accel_raw[2] << 4) & 0xF0; // aZ upper 4 bits
    ofst[8] = accel_raw[2] & 0xFF;  // aZ lower 8 bits
}