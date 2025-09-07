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


int cpp_main()
{   

	HAL_Delay(30000); //wait 30 seconds to clear the area

	HAL_GPIO_WritePin(PY1_GPIO_Port, PY1_Pin, GPIO_PIN_SET); //Trigger pyro channel 1
	HAL_Delay(50);
	HAL_GPIO_WritePin(PY1_GPIO_Port, PY1_Pin, GPIO_PIN_RESET);
	while (1)
	{   
		HAL_Delay(1000);
	}
}