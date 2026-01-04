#pragma once

#include "stm32h7xx_hal.h"
#include "MMC5983_Reg.h"

#define MMC5983_ADDR 0x30<<1 // MMC5983 I2C address

class MMC5983 {
    public:
        MMC5983(I2C_HandleTypeDef *hi2c, uint8_t bw, uint8_t cmodr, uint8_t prdset);

        HAL_StatusTypeDef Init();

        HAL_StatusTypeDef ReadMag();

        I2C_HandleTypeDef *hi2c; // I2C handle
        float mag_gauss[3]; // Magnetometer [X,Y,Z] in Gauss
        float temp; // Temperature in Celsius

        uint8_t testVar;


    private:
        HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *data, uint8_t len);
        HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t *data);
        HAL_StatusTypeDef WriteRegs(uint8_t reg, uint8_t *data, uint8_t len);

        uint8_t regDat;

        uint8_t rawDat[7]; // Raw data buffer 

        uint8_t bw;
        uint8_t cmodr;
        uint8_t prdset;
        float magConv = 0.0625E-3; // 0.0625 mG/LSB but we want Gauss

};

// Bandwidth Settings
enum MMC_BW {
    _100hz = 0,     // 8ms
    _200hz = 1,     // 4ms
    _400hz = 2,     // 2ms
    _800hz = 3      // 0.5ms
};

// Continuous Measurement Output Data Rate
enum MMC_CMODR {
    _off = 0,
    _1h = 1,
    _10h = 2,
    _20h = 3,
    _50h = 4,
    _100h = 5,
    _200h = 6,     // requires BW >= 200hz
    _1000h = 7     // requires BW = 800hz
};

// How many measurements between Set operations
enum MMC_PRDSET {
    _1 = 0,
    _25 = 1,
    _75 = 2,
    _100 = 3,
    _250 = 4,
    _500 = 5,
    _1000 = 6,
    _2000 = 7
};