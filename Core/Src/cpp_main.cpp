#include "cpp_main.h"
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fmac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"

#include "abstract.h"

#include "MMC5983.h"
#include "ICM42688.h"
#include "MS5607.h"
#include "ADXL375.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "SX1262.h"

#include <Eigen/Dense>
using namespace Eigen;

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint16_t usbTxBufLen;

MMC5983 mag(&hi2c3, MMC5983::_200Hz, MMC5983::_200hz, MMC5983::_75);
ICM42688 imu(&hi2c3, (float[3]){0,0,0}, (float[3]){0,0,0}, ICM42688::_2000dps, ICM42688::_200Hz, ICM42688::_16g, ICM42688::_200Hz);
MS5607 baro(&hi2c3, 0);
ADXL375 accel(&hi2c3, 100, (int8_t[3]){0,0,0});
SFE_UBLOX_GNSS myGNSS;


uint16_t bufLen = 0;

void print_matrix(Eigen::MatrixXd &X);

int cpp_main()
{   	
    SX1262_Get_st()->Busy_Pin = LoRa_BUSY_Pin;
	SX1262_Get_st()->Busy_Port= LoRa_BUSY_GPIO_Port;
	SX1262_Get_st()->NSS_Pin = LoRa_CS_Pin;
	SX1262_Get_st()->NSS_Port = LoRa_CS_GPIO_Port;
	SX1262_Get_st()->Reset_Pin = LoRa_RST_Pin;
	SX1262_Get_st()->Reset_Port = LoRa_RST_GPIO_Port;
    SX1262_Get_st()->SPI = LoRa_SPI_HANDLE;
	
	// uint8_t cmd[2] = {SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, SX1262_DIO3_OUTPUT_3_3};
	// HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	// HAL_SPI_Transmit(&LoRa_SPI_HANDLE,cmd,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	SX1262_Init();
    SX1262_SetFrequency(915e6);

    while (myGNSS.begin(hi2c3)==false) {
        SerialPrintln((uint8_t*)"GNSS I2C connection failed, retrying...");
        HAL_Delay(1000);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  
    myGNSS.setNavigationFrequency(20); // Solution rate in Hz (1-40Hz)
    myGNSS.setNavigationRate(1); // How many solutions to produce a measurement (1-127)
    
    myGNSS.setAutoPVT(true); // Tell the GNSS to output each solution periodically
    myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g); // Set the dynamic model to airborne 1G

	while (1)
	{   
		HAL_GPIO_WritePin(LoRa_PA_EN_GPIO_Port, LoRa_PA_EN_Pin, GPIO_PIN_SET);
		// HAL_Delay(100);
        SX1262_Transmit((uint8_t *)"Hello World!", 12);
		// HAL_Delay(100);
		HAL_GPIO_WritePin(LoRa_PA_EN_GPIO_Port, LoRa_PA_EN_Pin, GPIO_PIN_RESET);
        // SerialPrintln((uint8_t *)"Transmitted!");
        uint16_t bufLen = 0;
        volatile uint8_t SIV = myGNSS.getSIV();
        bufLen+=sprintf((char*)usbTxBuf, "\r\n>SIV: %d\n", SIV);

        volatile long latitude = myGNSS.getLatitude();
        bufLen+=sprintf((char*)usbTxBuf+bufLen, ">Lat: %.7f°N\n", (float)latitude/10000000.);

        volatile long longitude = myGNSS.getLongitude();
        bufLen+=sprintf((char*)usbTxBuf+bufLen, ">Long: %.7f°E\n", (float)longitude/10000000.);

        bufLen+=sprintf((char*)usbTxBuf+bufLen, ">Coords: %f:%f§°/1000|xy\n", (float)(latitude)/10000., (float)(longitude)/10000.);

        volatile long altitude = myGNSS.getAltitude();
        bufLen+=sprintf((char*)usbTxBuf+bufLen, ">Alt: %fm\n", (float)altitude/1000.);

        volatile long velN = myGNSS.getNedNorthVel();
        bufLen+=sprintf((char*)usbTxBuf+bufLen, ">VelN: %d mm/s\n", velN);

        volatile long velE = myGNSS.getNedEastVel();
        bufLen+=sprintf((char*)usbTxBuf+bufLen, ">VelE: %d mm/s\n", velE);

        SerialPrintln(usbTxBuf);
        HAL_Delay(1000);
		
	}
}

void print_matrix(Eigen::MatrixXd &X)  
{
    uint16_t bufLen = 0;

    uint8_t nrow = X.rows();
    uint8_t ncol = X.cols();
    for (uint8_t i=0; i<nrow; i++) {   
        bufLen+=sprintf((char*)usbTxBuf+bufLen, "[");

        for (uint8_t j=0; j<ncol; j++) {
            bufLen+=sprintf((char*)usbTxBuf+bufLen, "%.5f", X(i,j));

            if(j<ncol-1) bufLen+=sprintf((char*)usbTxBuf+bufLen, ",");
        }
        bufLen+=sprintf((char*)usbTxBuf+bufLen, "]\n");
    }
    SerialPrintln(usbTxBuf);
}