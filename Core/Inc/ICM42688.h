#pragma once

#include "stm32h7xx_hal.h"
#include "ICM42688_Reg.h"

#define ICM42688_ADDR 0x68<<1 // ICM42688 I2C address
#define ICM42688_DEVID 0x47 // WHO AM I value for ICM42688

#define ICM42688_REG_BANK_SEL 0x76 // Bank select register (0 - 4)

#define ICM42688_AAF_DELT 4
#define ICM42688_AAF_DELTSQR 16
#define ICM42688_AAF_BITSHIFT 11


class ICM42688 {
    public:
        ICM42688(I2C_HandleTypeDef *hi2c, float a_ofst[3], float g_ofst[3], uint8_t g_fs, uint8_t g_odr, uint8_t a_fs, uint8_t a_odr);

        HAL_StatusTypeDef Init();

        HAL_StatusTypeDef ReadIMU();

        I2C_HandleTypeDef *hi2c; // I2C handle
        float accel_ms2[3]; // Acceleration [X,Y,Z] in m/s^2
        float gyro_dps[3]; // Gyroscope [X,Y,Z] in degrees per second
        float temp; // Temperature in Celsius

        float accel_ofst[3]; // Offset values for each axis (pm 1g @ 0.5mg / LSB)
        float gyro_ofst[3]; // Offset values for each axis (pm 64dps @ 1/32dps / LSB)

        uint8_t testVar;


    private:
        HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *data, uint8_t len);
        HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t *data);
        HAL_StatusTypeDef WriteRegs(uint8_t reg, uint8_t *data, uint8_t len);
        HAL_StatusTypeDef setOfst();
        HAL_StatusTypeDef setFS_ODR();
        HAL_StatusTypeDef setAAF();
        HAL_StatusTypeDef selectBank(uint8_t bank);

        uint8_t regDat;


        uint8_t rawDat[14]; // Raw data buffer

        uint8_t g_fs;
        uint8_t g_odr;
        uint8_t a_fs;
        uint8_t a_odr;


        float accelConv;
        float gyroConv;

};

enum ICM_g_FS {
    _2000dps = (0 << 5),
    _1000dps = (1 << 5),
    _500dps = (2 << 5),
    _250dps = (3 << 5),
    _125dps = (4 << 5),
    _62_5dps = (5 << 5),
    _31_25dps = (6 << 5),
    _15_625dps = (7 << 5)
};

enum ICM_a_FS {
    _16g = (0 << 5),
    _8g = (1 << 5),
    _4g = (2 << 5),
    _2g = (3 << 5)
};

enum ICM_ODR {
    _32kHz = 1,   // 32kHz (LN mode)
    _16kHz = 2,   // 16kHz (LN mode)
    _8kHz = 3,    // 8kHz (LN mode)
    _4kHz = 4,    // 4kHz (LN mode)
    _2kHz = 5,    // 2kHz (LN mode)
    _1kHz = 6,    // 1kHz (LN mode) (default)
    _200Hz = 7,   // 200Hz (LP or LN mode)
    _100Hz = 8,   // 100Hz (LP or LN mode)
    _50Hz = 9,    // 50Hz (LP or LN mode)
    _25Hz = 10,   // 25Hz (LP or LN mode)
    _12_5Hz = 11, // 12.5Hz (LP or LN mode)
    _6_25Hz = 12, // 6.25Hz (LP mode)
    _3_125Hz = 13, // 3.125Hz (LP mode)
    _1_5625Hz = 14, // 1.5625Hz (LP mode)
    _500Hz = 15  // 500Hz (LP or LN mode)
};