#include "MMC5983.h"
#include "abstract.h"

MMC5983::MMC5983(I2C_HandleTypeDef *hi2c, uint8_t bw, uint8_t cmodr, uint8_t prdset) {
    this->hi2c = hi2c;
    this->bw = bw;
    this->cmodr = cmodr;
    this->prdset = prdset;
}

HAL_StatusTypeDef MMC5983::Init() {
    uint8_t status;
    status = ReadRegs(MMC5983_REG_PROD_ID, &regDat, 1);
    if (regDat != 0x30) 
        return HAL_ERROR; // Device ID mismatch

    regDat = 0b00001100; // send set command and enable automatic set/reset
    status |= WriteReg(MMC5983_REG_INTERNAL_0, &regDat);
    HAL_Delay(1);

    regDat = bw;  // set bandwidth
    status |= WriteReg(MMC5983_REG_INTERNAL_1, &regDat);

    regDat = cmodr;
    if(cmodr != 0) regDat |= 0b00001000; // enable continuous mode bit if not off
    regDat |= prdset << 4; // set periodic set frequency
    regDat |= 0b10000000; // enable periodic set
    status |= WriteReg(MMC5983_REG_INTERNAL_2, &regDat);
    return (HAL_StatusTypeDef)status;
}

/**
  * @brief  Read magnetometer data
  * @retval HAL Status
  */
HAL_StatusTypeDef MMC5983::ReadMag() {
    HAL_StatusTypeDef status;
    status = ReadRegs(MMC5983_REG_XOUT_0, rawDat, 7);
    // Convert raw data to float
    mag_gauss[0] = (((int32_t)(rawDat[0] << 10 | rawDat[1] << 2 | rawDat[7] >> 6))-131071.5f)*magConv;
    mag_gauss[1] = (((int32_t)(rawDat[2] << 10 | rawDat[3] << 2 | rawDat[7] >> 4))-131071.5f)*magConv;
    mag_gauss[2] = -(((int32_t)(rawDat[4] << 10 | rawDat[5] << 2 | rawDat[7] >> 2))-131071.5f)*magConv;

    return status;
}

/**
  * @brief  Reads Register(s) from the MMC5983
  * @param  reg: Register to read from
  * @param  *data: Pointer to the data buffer
  * @param  len: Number of bytes to read
  * @retval HAL Status
  */
 HAL_StatusTypeDef MMC5983::ReadRegs(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Read(hi2c, MMC5983_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Write Register to the MMC5983
  * @param  reg: Register to write to
  * @param  *data: Pointer to the data buffer
  * @retval HAL Status
  */
HAL_StatusTypeDef MMC5983::WriteReg(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(hi2c, MMC5983_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Write Registers to the MMC5983
  * @param  reg: Register to write to
  * @param  *data: Pointer to the data buffer
  * @param  len: Number of bytes to write
  * @retval HAL Status
  */
 HAL_StatusTypeDef MMC5983::WriteRegs(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Write(hi2c, MMC5983_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}