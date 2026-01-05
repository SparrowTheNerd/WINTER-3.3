#pragma once

#include "stm32h7xx_hal.h"

#define ADXL375_ADDR 0x53<<1 // ADXL375 I2C address
#define ADXL375_DEVID 0xE5 // ADXL375 device ID

#define ADXL375_SENSITIVITY 20.5   // g/LSB from datasheet, replace w calibrated data if desired
#define ADXL375_CONVERT 9.80665f / ADXL375_SENSITIVITY // conversion factor using sensitivity

class ADXL375 {
    public:
        ADXL375(I2C_HandleTypeDef *hi2c, uint8_t ODR, int8_t offset[3]);
        
        HAL_StatusTypeDef Init();
        HAL_StatusTypeDef ReadAccel();

        I2C_HandleTypeDef *hi2c; // I2C handle
        float accel_ms2[3]; // Acceleration [X,Y,Z] in m/s^2
        uint8_t ODR;

    private:
        /*
        * LOW LEVEL FUNCTIONS
        */
        HAL_StatusTypeDef ReadReg(uint8_t reg, uint8_t *data, uint8_t len);
        HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t *data);

        float ofst[3]; // Offset values for each axis
};
/*
* REGISTERS
*/
#define ADXL375_REG_DEVID               0x00 // Device ID register
#define ADXL375_REG_THRESH_SHOCK        0x1D // Shock threshold register
#define ADXL375_REG_OFSX                0x1E // X-axis offset register
#define ADXL375_REG_OFSY                0x1F // Y-axis offset register
#define ADXL375_REG_OFSZ                0x20 // Z-axis offset register
#define ADXL375_REG_DUR                 0x21 // Shock duration register
#define ADXL375_REG_Latent              0x22 // Shock latency register
#define ADXL375_REG_Window              0x23 // Shock window register
#define ADXL375_REG_THRESH_ACT          0x24 // Activity threshold register
#define ADXL375_REG_THRESH_INACT        0x25 // Inactivity threshold register
#define ADXL375_REG_TIME_INACT          0x26 // Inactivity time register
#define ADXL375_REG_ACT_INACT_CTL       0x27 // Activity/Inactivity control register
#define ADXL375_REG_SHOCK_AXES          0x28 // Shock axes control register
#define ADXL375_REG_ACT_SHOCK_STATUS    0x29 // Source of single/double shock register
#define ADXL375_REG_BW_RATE             0x2C // Data rate and power mode control register
#define ADXL375_REG_POWER_CTL           0x2D // Power control register
#define ADXL375_REG_INT_ENABLE          0x2E // Interrupt enable register
#define ADXL375_REG_INT_MAP             0x2F // Interrupt mapping register
#define ADXL375_REG_INT_SOURCE          0x30 // Interrupt source register
#define ADXL375_REG_DATA_FORMAT         0x31 // Data format register
#define ADXL375_REG_DATAX0              0x32 // X-axis data register (LSB)
#define ADXL375_REG_DATAX1              0x33 // X-axis data register (MSB)
#define ADXL375_REG_DATAY0              0x34 // Y-axis data register (LSB)
#define ADXL375_REG_DATAY1              0x35 // Y-axis data register (MSB)
#define ADXL375_REG_DATAZ0              0x36 // Z-axis data register (LSB)
#define ADXL375_REG_DATAZ1              0x37 // Z-axis data register (MSB)
#define ADXL375_REG_FIFO_CTL            0x38 // FIFO control register
#define ADXL375_REG_FIFO_STATUS         0x39 // FIFO status register