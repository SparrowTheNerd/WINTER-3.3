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

#include <Eigen/Dense>
using namespace Eigen;

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint16_t usbTxBufLen;

void print_matrix(MatrixXf &X);


int cpp_main()
{   	
    HAL_Delay(2000);
    SerialPrintln((uint8_t*)"Eigen Test:");
	MatrixXf m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    print_matrix(m);

	while (1)
	{   
		
		HAL_Delay(1000);
	}
}

void print_matrix(MatrixXf &X)  
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