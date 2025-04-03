#ifndef __ICM42688_H
#define __ICM42688_H

#include "stm32h7xx_hal.h"

#define ICM42688_ADDR 0x68<<1 // ICM42688 I2C address
#define ICM42688_DEVID 0x47 // WHO AM I value for ICM42688

#define ICM42688_REG_BANK_SEL 0x76 // Bank select register (0 - 4)

typedef struct {

} ICM42688_rawDat;

typedef struct {
    I2C_HandleTypeDef *hi2c; // I2C handle
    float accel_ms2[3]; // Acceleration [X,Y,Z] in m/s^2
    float gyro_dps[3]; // Gyroscope [X,Y,Z] in degrees per second
    float accel_ofst[3]; // Offset values for each axis (pm 1g @ 0.5mg / LSB)
    float gyro_ofst[3]; // Offset values for each axis (pm 64dps @ 1/32dps / LSB)
    float temp; // Temperature in Celsius
    ICM42688_rawDat raw; // Raw data structure
} ICM42688;

typedef enum
{
    _2000dps = (0 << 5),
    _1000dps = (1 << 5),
    _500dps = (2 << 5),
    _250dps = (3 << 5),
    _125dps = (4 << 5),
    _62_5dps = (5 << 5),
    _31_25dps = (6 << 5),
    _15_625dps = (7 << 5)
} ICM42688_gyro_FS;

typedef enum {
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
} ICM42688_gyro_ODR;

typedef enum
{
	_16g = (0 << 5),
    _8g = (1 << 5),
    _4g = (2 << 5),
    _2g = (3 << 5)
} ICM42688_accel_FS;

typedef enum {
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
} ICM42688_accel_ODR;



HAL_StatusTypeDef ICM42688_Init(ICM42688 *icm, I2C_HandleTypeDef *hi2c, float a_ofst[3], float g_ofst[3], uint8_t g_fs, uint8_t g_odr, uint8_t a_fs, uint8_t a_odr);
HAL_StatusTypeDef ICM42688_ReadRegs(ICM42688 *icm, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef ICM42688_WriteReg(ICM42688 *icm, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ICM42688_WriteRegs(ICM42688 *icm, uint8_t reg, uint8_t *data, uint8_t len);

void convertOfst(ICM42688 *icm, int8_t ofst[9]);

/*
* BANK 0 Registers
*/
#define ICM42688_REG_B0_DEVICE_CONFIG       0x11
#define ICM42688_REG_B0_DRIVE_CONFIG	    0x13
#define ICM42688_REG_B0_INT_CONFIG	        0x14
#define ICM42688_REG_B0_FIFO_CONFIG	        0x16
#define ICM42688_REG_B0_TEMP_DATA1	        0x1D
#define ICM42688_REG_B0_TEMP_DATA0	        0x1E
#define ICM42688_REG_B0_ACCEL_DATA_X1	    0x1F
#define ICM42688_REG_B0_ACCEL_DATA_X0	    0x20
#define ICM42688_REG_B0_ACCEL_DATA_Y1	    0x21
#define ICM42688_REG_B0_ACCEL_DATA_Y0	    0x22
#define ICM42688_REG_B0_ACCEL_DATA_Z1	    0x23
#define ICM42688_REG_B0_ACCEL_DATA_Z0	    0x24
#define ICM42688_REG_B0_GYRO_DATA_X1	    0x25
#define ICM42688_REG_B0_GYRO_DATA_X0	    0x26
#define ICM42688_REG_B0_GYRO_DATA_Y1	    0x27
#define ICM42688_REG_B0_GYRO_DATA_Y0	    0x28
#define ICM42688_REG_B0_GYRO_DATA_Z1	    0x29
#define ICM42688_REG_B0_GYRO_DATA_Z0	    0x2A
#define ICM42688_REG_B0_TMST_FSYNCH	        0x2B
#define ICM42688_REG_B0_TMST_FSYNCL	        0x2C
#define ICM42688_REG_B0_INT_STATUS	        0x2D
#define ICM42688_REG_B0_FIFO_COUNTH	        0x2E
#define ICM42688_REG_B0_FIFO_COUNTL	        0x2F
#define ICM42688_REG_B0_FIFO_DATA	        0x30
#define ICM42688_REG_B0_APEX_DATA0	        0x31
#define ICM42688_REG_B0_APEX_DATA1	        0x32
#define ICM42688_REG_B0_APEX_DATA2	        0x33
#define ICM42688_REG_B0_APEX_DATA3	        0x34
#define ICM42688_REG_B0_APEX_DATA4	        0x35
#define ICM42688_REG_B0_APEX_DATA5	        0x36
#define ICM42688_REG_B0_INT_STATUS2	        0x37
#define ICM42688_REG_B0_INT_STATUS3	        0x38
#define ICM42688_REG_B0_SIGNAL_PATH_RESET	0x4B
#define ICM42688_REG_B0_INTF_CONFIG0	    0x4C
#define ICM42688_REG_B0_INTF_CONFIG1	    0x4D
#define ICM42688_REG_B0_PWR_MGMTO	        0x4E
#define ICM42688_REG_B0_GYRO_CONFIG0	    0x4F
#define ICM42688_REG_B0_ACCEL_CONFIG0	    0x50
#define ICM42688_REG_B0_GYRO_CONFIG1	    0x51
#define ICM42688_REG_B0_GYRO_ACCEL_CONFIG0	0x52
#define ICM42688_REG_B0_ACCEL_CONFIG1	    0x53
#define ICM42688_REG_B0_TMST_CONFIG	        0x54
#define ICM42688_REG_B0_APEX_CONFIG0	    0x56
#define ICM42688_REG_B0_SMD_CONFIG	        0x57
#define ICM42688_REG_B0_FIFO_CONFIG1	    0x5F
#define ICM42688_REG_B0_FIFO_CONFIG2	    0x60
#define ICM42688_REG_B0_FIFO_CONFIG3	    0x61
#define ICM42688_REG_B0_FSYNC_CONFIG	    0x62
#define ICM42688_REG_B0_INT_CONFIG0	        0x63
#define ICM42688_REG_B0_INT_CONFIG1	        0x64
#define ICM42688_REG_B0_INT_SOURCE0	        0x65
#define ICM42688_REG_B0_INT_SOURCE1	        0x66
#define ICM42688_REG_B0_INT_SOURCE3	        0x68
#define ICM42688_REG_B0_INT_SOURCE4	        0x69
#define ICM42688_REG_B0_FIFO_LOST_PKT0	    0x6C
#define ICM42688_REG_B0_FIFO_LOST_PKT1	    0x6D
#define ICM42688_REG_B0_SELF_TEST_CONFIG	0x70
#define ICM42688_REG_B0_WHO_AM_I	        0x75


/*
* BANK 1 Registers
*/
#define ICM42688_REG_B1_SENSOR_CONFIG0	        0x03
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC2	    0x0B
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC3	    0x0C
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC4	    0x0D
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC5	    0x0E
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC6	    0x0F
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC7	    0x10
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC8	    0x11
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC9	    0x12
#define ICM42688_REG_B1_GYRO_CONFIG_STATIC10	0x13
#define ICM42688_REG_B1_XG_ST_DATA	            0x5F
#define ICM42688_REG_B1_YG_ST_DATA	            0x60
#define ICM42688_REG_B1_ZG_ST_DATA	            0x61
#define ICM42688_REG_B1_TMSTVAL0	            0x62
#define ICM42688_REG_B1_TMSTVAL1	            0x63
#define ICM42688_REG_B1_TMSTVAL2	            0x64
#define ICM42688_REG_B1_INTF_CONFIG4	        0x7A
#define ICM42688_REG_B1_INTF_CONFIG5	        0x7B
#define ICM42688_REG_B1_INTF_CONFIG6	        0x7C

/*
* BANK 2 Registers
*/
#define ICM42688_REG_B2_ACCEL_CONFIG_STATIC2	0x03
#define ICM42688_REG_B2_ACCEL_CONFIG_STATIC3	0x04
#define ICM42688_REG_B2_ACCEL_CONFIG_STATIC4	0x05
#define ICM42688_REG_B2_XA_ST_DATA	            0x3B
#define ICM42688_REG_B2_YA_ST_DATA	            0x3C
#define ICM42688_REG_B2_ZA_ST_DATA	            0x3D

/*
* BANK 3 Registers
*/
#define ICM42688_REG_B3_CLKDIV                  0x2A

/*
* BANK 4 Registers
*/
#define ICM42688_REG_B4_APEX_CONFIG1	        0x40
#define ICM42688_REG_B4_APEX_CONFIG2	        0x41
#define ICM42688_REG_B4_APEX_CONFIG3	        0x42
#define ICM42688_REG_B4_APEX_CONFIG4	        0x43
#define ICM42688_REG_B4_APEX_CONFIG5	        0x44
#define ICM42688_REG_B4_APEX_CONFIG6	        0x45
#define ICM42688_REG_B4_APEX_CONFIG7	        0x46
#define ICM42688_REG_B4_APEX_CONFIG8	        0x47
#define ICM42688_REG_B4_APEX_CONFIG9	        0x48
#define ICM42688_REG_B4_ACCEL_WOM_X_THR	        0x4A
#define ICM42688_REG_B4_ACCEL_WOM_Y_THR	        0x4B
#define ICM42688_REG_B4_ACCEL_WOM_Z_THR	        0x40
#define ICM42688_REG_B4_INT_SOURCE6	            0x4D
#define ICM42688_REG_B4_INT_SOURCE7	            0x4E
#define ICM42688_REG_B4_INT_SOURCE8	            0x4F
#define ICM42688_REG_B4_INT_SOURCE9	            0x50
#define ICM42688_REG_B4_INT_SOURCE10	        0x51
#define ICM42688_REG_B4_OFFSET_USER0	        0x77
#define ICM42688_REG_B4_OFFSET_USER1	        0x78
#define ICM42688_REG_B4_OFFSET_USER2	        0x79
#define ICM42688_REG_B4_OFFSET_USER3	        0x7A
#define ICM42688_REG_B4_OFFSET_USER4	        0x7B
#define ICM42688_REG_B4_OFFSET_USER5	        0x7C
#define ICM42688_REG_B4_OFFSET_USER6	        0x7D
#define ICM42688_REG_B4_OFFSET_USER7	        0x7E
#define ICM42688_REG_B4_OFFSET_USER8	        0x7F

#endif /* __ICM42688_H */