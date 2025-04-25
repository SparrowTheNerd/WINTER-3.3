#include "ICM42688.h"

ICM42688::ICM42688(I2C_HandleTypeDef *hi2c, float a_ofst[3], float g_ofst[3], uint8_t g_fs, uint8_t g_odr, uint8_t a_fs, uint8_t a_odr) {
    this->hi2c = hi2c;
    this->accel_ofst[0] = a_ofst[0];
    this->accel_ofst[1] = a_ofst[1];
    this->accel_ofst[2] = a_ofst[2];
    this->gyro_ofst[0] = g_ofst[0];
    this->gyro_ofst[1] = g_ofst[1];
    this->gyro_ofst[2] = g_ofst[2];
    this->g_fs = g_fs;
    this->g_odr = g_odr;
    this->a_fs = a_fs;
    this->a_odr = a_odr;

    switch(a_fs) {  //g/LSB
        case _2g:
            accelConv = 1.f/16384.f;
            break;
        case _4g:
            accelConv = 1.f/8192.f; 
            break;
        case _8g:
            accelConv = 1.f/4096.f;
            break;
        case _16g:
            accelConv = 1.f/2048.f;
            break;
    }
    accelConv *= 9.80665f; // Convert to m/s^2

    switch(g_fs) {  //dps/LSB
        case _15_625dps:
            gyroConv = 1.f/2097.2f;
            break;
        case _31_25dps:
            gyroConv = 1.f/1048.6f;
            break;
        case _62_5dps:
            gyroConv = 1.f/524.3f;
            break;
        case _125dps:
            gyroConv = 1.f/262.15f;
            break;
        case _250dps:
            gyroConv = 1.f/131.075f;
            break;
        case _500dps:
            gyroConv = 1.f/65.5375f;
            break;
        case _1000dps:
            gyroConv = 1.f/32.76875f;
            break;
        case _2000dps:
            gyroConv = 1.f/16.384375f;
            break;
    }

}

HAL_StatusTypeDef ICM42688::Init() {
    // configure for i2c
    uint8_t status = selectBank(1); // Select Bank 1
    regDat = 0x5C;
    status |= WriteReg(ICM42688_REG_B1_INTF_CONFIG6, &regDat);

    status |= selectBank(0); // Select Bank 0
    regDat = 0x09;
    status |= WriteReg(ICM42688_REG_B0_DRIVE_CONFIG, &regDat); // Set drive config to 0x09 (default)  

    status = ReadRegs(ICM42688_REG_B0_WHO_AM_I, &regDat, 1);
    if (regDat != ICM42688_DEVID) {
        return HAL_ERROR; // Device ID mismatch
    }
    
    status |= setFS_ODR();

    status |= setOfst(); // Set offsets

    status |= setAAF();

    regDat = 0x0F; // Set to 0x0F for normal operation
    status |= selectBank(0);
    status |= WriteReg(ICM42688_REG_B0_PWR_MGMTO, &regDat); // Set power management to normal operation

    return (HAL_StatusTypeDef)status;
}

/**
  * @brief  Read IMU data
  * @retval HAL Status
  */
HAL_StatusTypeDef ICM42688::ReadIMU() {
    uint8_t status = selectBank(0);
    status |= ReadRegs(ICM42688_REG_B0_TEMP_DATA1, rawDat, 14);

    // Convert raw data to float
    temp = ((int16_t)(rawDat[0] << 8 | rawDat[1])) / 132.48f + 25.0f;


    for(int i=0; i<3; i++) {
        uint8_t aI = i * 2 + 2; // 2 bytes per axis
        uint8_t gI = i * 2 + 8; // 2 bytes per axis
        accel_ms2[i] = ((int16_t)(rawDat[aI] << 8 | rawDat[aI + 1])) * accelConv;
        gyro_dps[i] = ((int16_t)(rawDat[gI] << 8 | rawDat[gI + 1])) * gyroConv;
    }

    // accel_ms2[0] = ((int16_t)(rawDat[2] << 8 | rawDat[3])) * accelConv;
    
    return (HAL_StatusTypeDef)status;
}

/**
  * @brief  Set full scale and output data rate
  * @retval HAL Status
  */
 HAL_StatusTypeDef ICM42688::setFS_ODR() {
    uint8_t status = selectBank(0);
    regDat = g_fs | g_odr;    
    status |= WriteReg(ICM42688_REG_B0_GYRO_CONFIG0, &regDat);
    regDat = a_fs | a_odr;
    return (HAL_StatusTypeDef)(status | WriteReg(ICM42688_REG_B0_ACCEL_CONFIG0, &regDat));
}

/**
  * @brief Configure anti-aliasing filter
  * @retval HAL Status
  * @note Consult datasheet section 5.3 for details
  */
HAL_StatusTypeDef ICM42688::setAAF() {
    uint8_t status = selectBank(2);
    // a
    regDat = ICM42688_AAF_DELT << 1;
    status |= WriteReg(ICM42688_REG_B2_ACCEL_CONFIG_STATIC2, &regDat);
    // b
    regDat = ICM42688_AAF_DELTSQR;
    status |= WriteReg(ICM42688_REG_B2_ACCEL_CONFIG_STATIC3, &regDat);
    // c
    regDat = ICM42688_AAF_BITSHIFT << 4 | (uint8_t)((uint16_t)ICM42688_AAF_DELTSQR >> 8);
    status |= WriteReg(ICM42688_REG_B2_ACCEL_CONFIG_STATIC4, &regDat);

    status |= selectBank(1);
    // d
    regDat = ICM42688_AAF_DELT;
    status |= WriteReg(ICM42688_REG_B1_GYRO_CONFIG_STATIC3, &regDat);
    // e
    regDat = ICM42688_AAF_DELTSQR;
    status |= WriteReg(ICM42688_REG_B1_GYRO_CONFIG_STATIC4, &regDat);
    // f
    regDat = ICM42688_AAF_BITSHIFT << 4 | (uint8_t)((uint16_t)ICM42688_AAF_DELTSQR >> 8);
    status |= WriteReg(ICM42688_REG_B1_GYRO_CONFIG_STATIC5, &regDat);

    return (HAL_StatusTypeDef)status;
}

/**
  * @brief  Convert offsets and write register values
  * @retval None
  */
 HAL_StatusTypeDef ICM42688::setOfst() {
    int8_t ofst[9];
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

    // Write the offsets to the registers
    return WriteRegs(ICM42688_REG_B4_OFFSET_USER0, (uint8_t*)ofst, 9);
}

HAL_StatusTypeDef ICM42688::selectBank(uint8_t bank) {
    return WriteReg(ICM42688_REG_BANK_SEL, &bank);
}

/**
  * @brief  Reads Register(s) from the ICM42688
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
  * @param  reg: Register to write to
  * @param  *data: Pointer to the data buffer
  * @retval HAL Status
  */
HAL_StatusTypeDef ICM42688::WriteReg(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(hi2c, ICM42688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Write Registers to the ICM42688
  * @param  reg: Register to write to
  * @param  *data: Pointer to the data buffer
  * @param  len: Number of bytes to write
  * @retval HAL Status
  */
 HAL_StatusTypeDef ICM42688::WriteRegs(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Write(hi2c, ICM42688_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}