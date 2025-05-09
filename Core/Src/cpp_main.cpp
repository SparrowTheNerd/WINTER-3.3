#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fmac.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "cpp_main.h"
#include "usbd_cdc_if.h"
#include "abstract.h"
#include "ADXL375.h"
#include "MS5607.h"
#include "ICM42688.h"

#define USBBUF_MAXLEN 256

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint8_t i2cBuf;
uint16_t usbTxBufLen;
int8_t highGOfst[3] = {0,0,0};
float imuOfst[3] = {0,0,0};

int cpp_main()
{
    ADXL375 highG = ADXL375(&hi2c3, 10, highGOfst); // 10Hz ODR
    MS5607 baro = MS5607(&hi2c3, 4); // 4096 OSR
    ICM42688 imu = ICM42688(&hi2c3, imuOfst, imuOfst, _2000dps, _100Hz, _16g, _100Hz);  //full scale, 100hz

	/* Initialization */
    HAL_Delay(100);
    HAL_StatusTypeDef status = highG.Init();
    if (status != HAL_OK) {
        SerialPrintln((uint8_t*)"ADXL375 Init Failed!");
        while(1){
            SerialPrintln((uint8_t*)"ADXL375 Init Failed!");
            HAL_Delay(1000);
        }
    } else {
        SerialPrintln((uint8_t*)"ADXL375 Init Success!");
    }

    status = baro.Init(); // 4096 OSR
    if (status != HAL_OK) {
        SerialPrintln((uint8_t*)"MS5607 Init Failed!");
        while(1){
            SerialPrintln((uint8_t*)"MS5607 Init Failed!");
            HAL_Delay(1000);
        }
    } else {
        SerialPrintln((uint8_t*)"MS5607 Init Success!");
    }

    status = imu.Init();
    if (status != HAL_OK) {
        SerialPrintln((uint8_t*)"ICM42688 Init Failed!");
        while(1){
            SerialPrintln((uint8_t*)"ICM42688 Init Failed!");
            HAL_Delay(1000);
        }
    } else {
        SerialPrintln((uint8_t*)"ICM42688 Init Success!");
    }

	while (1)
	{
		HAL_StatusTypeDef status = imu.ReadIMU();
        if (status != HAL_OK) {
            SerialPrintln((uint8_t*)"ICM42688 Read Failed!");
            HAL_Delay(1000);
        } else {
            sprintf((char*)usbTxBuf, ">aX:%.3f\n>aY:%.3f\n>aZ:%.3f\n>gX:%.3f\n>gY:%.3f\n>gZ:%.3f", imu.accel_ms2[0], imu.accel_ms2[1], imu.accel_ms2[2], imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2]);
            // sprintf((char*)usbTxBuf,"aX: %.3f m/s2",imu.accel_ms2[0]);
            SerialPrintln(usbTxBuf);
        }
        HAL_Delay(10);
	}
}