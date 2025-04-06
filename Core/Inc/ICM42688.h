#pragma once

#include "stm32h7xx_hal.h"
#include "ICM42688_Reg.h"

#define ICM42688_ADDR 0x68<<1 // ICM42688 I2C address
#define ICM42688_DEVID 0x47 // WHO AM I value for ICM42688

#define ICM42688_REG_BANK_SEL 0x76 // Bank select register (0 - 4)


class ICM42688 {
    public:
        ICM42688(I2C_HandleTypeDef *hi2c, float a_ofst[3], float g_ofst[3], uint8_t g_fs, uint8_t g_odr, uint8_t a_fs, uint8_t a_odr);

        HAL_StatusTypeDef Init();

        I2C_HandleTypeDef *hi2c; // I2C handle
        float accel_ms2[3]; // Acceleration [X,Y,Z] in m/s^2
        float gyro_dps[3]; // Gyroscope [X,Y,Z] in degrees per second
        float accel_ofst[3]; // Offset values for each axis (pm 1g @ 0.5mg / LSB)
        float gyro_ofst[3]; // Offset values for each axis (pm 64dps @ 1/32dps / LSB)
        float temp; // Temperature in Celsius
        
        enum g_FS {
            _2000dps = (0 << 5),
            _1000dps = (1 << 5),
            _500dps = (2 << 5),
            _250dps = (3 << 5),
            _125dps = (4 << 5),
            _62_5dps = (5 << 5),
            _31_25dps = (6 << 5),
            _15_625dps = (7 << 5)
        };

        enum g_ODR {
            GODR_32kHz = 1,   // 32kHz
            GODR_16kHz = 2,   // 16kHz
            GODR_8kHz  = 3,   // 8kHz 
            GODR_4kHz  = 4,   // 4kHz 
            GODR_2kHz  = 5,   // 2kHz 
            GODR_1kHz  = 6,   // 1kHz 
            GODR_200Hz = 7,   // 200Hz
            GODR_100Hz = 8,   // 100Hz
            GODR_50Hz  = 9,   // 50Hz 
            GODR_25Hz  = 10,  // 25Hz 
            GODR_12_5Hz = 11, // 12.5Hz
            GODR_500Hz = 15   // 500Hz
        };

        enum a_FS {
            _16g = (0 << 5),
            _8g = (1 << 5),
            _4g = (2 << 5),
            _2g = (3 << 5)
        };

        enum a_ODR {
            AODR_32kHz = 1,   // 32kHz (LN mode)
            AODR_16kHz = 2,   // 16kHz (LN mode)
            AODR_8kHz = 3,    // 8kHz (LN mode)
            AODR_4kHz = 4,    // 4kHz (LN mode)
            AODR_2kHz = 5,    // 2kHz (LN mode)
            AODR_1kHz = 6,    // 1kHz (LN mode) (default)
            AODR_200Hz = 7,   // 200Hz (LP or LN mode)
            AODR_100Hz = 8,   // 100Hz (LP or LN mode)
            AODR_50Hz = 9,    // 50Hz (LP or LN mode)
            AODR_25Hz = 10,   // 25Hz (LP or LN mode)
            AODR_12_5Hz = 11, // 12.5Hz (LP or LN mode)
            AODR_6_25Hz = 12, // 6.25Hz (LP mode)
            AODR_3_125Hz = 13, // 3.125Hz (LP mode)
            AODR_1_5625Hz = 14, // 1.5625Hz (LP mode)
            AODR_500Hz = 15  // 500Hz (LP or LN mode)
        };

    private:
        HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *data, uint8_t len);
        HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t *data);
        HAL_StatusTypeDef WriteRegs(uint8_t reg, uint8_t *data, uint8_t len);
        void convertOfst(int8_t ofst[9]);

        struct rawDat {

        };

        rawDat raw; // Raw data structure

};