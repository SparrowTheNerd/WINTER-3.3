#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fmac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "cpp_main.h"
#include "usbd_cdc_if.h"
#include "abstract.h"

#include "SparkFun_u-blox_GNSS_v3.h"

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint16_t usbTxBufLen;

SFE_UBLOX_GNSS myGNSS;


int cpp_main()
{   	

	SerialPrintln((uint8_t*)"\r\n\n--- GNSS I2C Connection Test ---");

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
		
		HAL_Delay(50);
	}
}