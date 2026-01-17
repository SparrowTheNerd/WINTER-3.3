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

uint8_t usbTxBuf[USBBUF_MAXLEN];

MMC5983 mag(&hi2c3, MMC5983::_200Hz, MMC5983::_200hz, MMC5983::_75);

int cpp_main()
{   	
    HAL_Delay(1000);
    // Initialize sensors
    if(mag.Init()!=HAL_OK) {
        while(1) {
            SerialPrintln((uint8_t*)"Mag init failed");
            HAL_Delay(1000);
        }
    }

	while (1)
	{   
		mag.ReadMag();
        sprintf((char*)usbTxBuf, "Raw:0,0,0,0,0,0,%d,%d,%d",
            (int32_t)(mag.mag_gauss(0)*1000.f),
            (int32_t)(mag.mag_gauss(1)*1000.f),
            (int32_t)(mag.mag_gauss(2)*1000.f)
        );
        SerialPrintln(usbTxBuf);
		HAL_Delay(100);
	}
}