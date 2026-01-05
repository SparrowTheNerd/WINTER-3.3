#pragma once

#include "stm32h7xx_hal.h"
#include "MMC5983_Reg.h"
#include "Eigen/Dense"
using namespace Eigen;

#define MMC5983_ADDR 0x30<<1 // MMC5983 I2C address

class MMC5983 {
    public:
        MMC5983(I2C_HandleTypeDef *hi2c, uint8_t bw, uint8_t cmodr, uint8_t prdset);

        HAL_StatusTypeDef Init();

        HAL_StatusTypeDef ReadMag();

        I2C_HandleTypeDef *hi2c; // I2C handle
        Vector3d mag_gauss; // Magnetometer in Eigen vector
        float temp; // Temperature in Celsius

        uint8_t testVar;

        // Bandwidth Settings
        enum MMC_BW {
            _100Hz = 0,     // 8ms
            _200Hz = 1,     // 4ms
            _400Hz = 2,     // 2ms
            _800Hz = 3      // 0.5ms
        };

        // Continuous Measurement Output Data Rate
        enum MMC_CMODR {
            _off = 0,
            _1hz = 1,
            _10hz = 2,
            _20hz = 3,
            _50hz = 4,
            _100hz = 5,
            _200hz = 6,     // requires BW >= 200hz
            _1000hz = 7     // requires BW = 800hz
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

        Vector3d magHardIronOffset {0.0697f, -0.0246f, -0.6230f};   // Hard iron offset in Gauss
        Matrix3d magSoftIronOffset {
            {0.964f,  0.003f, -0.016f},
            {0.003f,  0.999f,  0.025f},
            {-0.016f, 0.025f, 1.040f}
        };

};