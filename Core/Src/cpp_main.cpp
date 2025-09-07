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
    SX1262_Get_st()->Busy_Pin = LoRa_BUSY_Pin;
	SX1262_Get_st()->Busy_Port= LoRa_BUSY_GPIO_Port;
	SX1262_Get_st()->NSS_Pin = LoRa_CS_Pin;
	SX1262_Get_st()->NSS_Port = LoRa_CS_GPIO_Port;
	SX1262_Get_st()->Reset_Pin = LoRa_RST_Pin;
	SX1262_Get_st()->Reset_Port = LoRa_RST_GPIO_Port;
    SX1262_Get_st()->SPI = LoRa_SPI_HANDLE;
	
	uint8_t cmd[2] = {SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, SX1262_DIO3_OUTPUT_3_3};
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&LoRa_SPI_HANDLE,cmd,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	SX1262_Init();
    SX1262_SetFrequency(915e6);

	while (1)
	{   
		HAL_GPIO_WritePin(LoRa_PA_EN_GPIO_Port, LoRa_PA_EN_Pin, GPIO_PIN_SET);
		// HAL_Delay(100);
        SX1262_Transmit((uint8_t *)"Hello World!", 12);
		// HAL_Delay(100);
		HAL_GPIO_WritePin(LoRa_PA_EN_GPIO_Port, LoRa_PA_EN_Pin, GPIO_PIN_RESET);
        SerialPrintln((uint8_t *)"Transmitted!");
        HAL_Delay(1000);
		
	}
}