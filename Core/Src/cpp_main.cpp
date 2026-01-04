#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fmac.h"
#include "i2c.h"
// #include "memorymap.h"
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
#include "MMC5983.h"
#include "../../SparkFun-UBlox-STM32/src/SparkFun_u-blox_GNSS_v3.h"
#include "SX1262.h"

#include "fatfs.h"
#include "user_diskio.h"
#include <string.h>
#include <stdio.h>

/* 
 * __attribute__((section(".nocache")));
 */

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint8_t i2cBuf;
uint16_t usbTxBufLen;
int8_t highGOfst[3] = {0,0,0};
float imuOfst[3] = {0,0,0};

SFE_UBLOX_GNSS myGNSS;
ICM42688 IMU(&hi2c3, imuOfst, imuOfst, ICM_g_FS::_2000dps, ICM_ODR::_500Hz, ICM_a_FS::_16g, ICM_ODR::_500Hz);
ADXL375 HighG(&hi2c3, 12, highGOfst);
MMC5983 Mag(&hi2c3, MMC_BW::_100hz, MMC_CMODR::_100h, MMC_PRDSET::_250);


int cpp_main()
{ 

    if(Mag.Init() == HAL_ERROR) {
        while(1) {
            SerialPrintln((uint8_t*)">MMC5983 init failed!\n");
            HAL_Delay(1000);
        }
    }

	while (1)
	{   

        Mag.ReadMag();
        sprintf((char*)usbTxBuf, ">mX: %.6f\n>mY: %.6f\n>mZ: %.6f\n", Mag.mag_gauss[0], Mag.mag_gauss[1], Mag.mag_gauss[2]);
        SerialPrintln(usbTxBuf);
		
		HAL_Delay(10);
	}
}