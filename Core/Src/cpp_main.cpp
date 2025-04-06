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

int cpp_main()
{
    ADXL375 highG = ADXL375(&hi2c3, 10, highGOfst); // 10Hz ODR
    MS5607 baro = MS5607(&hi2c3, 4); // 4096 OSR

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

	while (1)
	{
		/* Super loop */
	}
}