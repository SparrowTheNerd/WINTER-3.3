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

#include "kalman.h"

#include <Eigen/Dense>
using namespace Eigen;

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint16_t usbTxBufLen;

MMC5983 mag(&hi2c3, MMC5983::_200Hz, MMC5983::_200hz, MMC5983::_75);
ICM42688 imu(&hi2c3, (float[3]){0,0,0}, (float[3]){0,0,0}, ICM42688::_1000dps, ICM42688::_200Hz, ICM42688::_16g, ICM42688::_200Hz);
MS5607 baro(&hi2c3, 0);
ADXL375 highG(&hi2c3, 100, (int8_t[3]){0,0,0});
SFE_UBLOX_GNSS myGNSS;

KalmanFilter ekf;


uint16_t bufLen = 0;

void print_matrix(Eigen::MatrixXd &X);

double prevTime;
double dt;

int cpp_main()
{   	
    while (myGNSS.begin(hi2c3)==false) {
        SerialPrintln((uint8_t*)"GNSS I2C connection failed, retrying...");
        HAL_Delay(1000);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  
    myGNSS.setNavigationFrequency(20); // Solution rate in Hz (1-40Hz)
    myGNSS.setNavigationRate(1); // How many solutions to produce a measurement (1-127)
    
    myGNSS.setAutoPVT(true); // Tell the GNSS to output each solution periodically
    myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g); // Set the dynamic model to airborne 1G

    HAL_Delay(1000);

    while(imu.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"IMU Init Failed"); HAL_Delay(1000);
    }
    while(mag.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"Mag Init Failed"); HAL_Delay(1000);
    }
    while(highG.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"High-G Accel Init Failed"); HAL_Delay(1000);
    }
    HAL_Delay(5000);

    ekf.init(&mag, &imu, &baro, &highG, &myGNSS);
    prevTime = (double)(HAL_GetTick())/1000.0;
    uint8_t printCounter = 0;
	while (1)
	{   
        dt = (double)(HAL_GetTick())/1000.0 - prevTime;
		ekf.predict(dt);
        prevTime = (double)(HAL_GetTick())/1000.0;
        printCounter++;
        HAL_Delay(10);
        // sprintf((char*)usbTxBuf, "%.5f \t %.5f \t %.5f \t %.5f", ekf.intState(0), ekf.intState(1), ekf.intState(2), ekf.intState(3));
        if(printCounter > 3) {
            sprintf((char*)usbTxBuf, ">3D|Orientation:S:cube:P:0:0:0:Q:%.5f:%.5f:%.5f:%.5f:W:1:H:1:D:1:C:#ff0000", ekf.intState(1), ekf.intState(2), ekf.intState(3), ekf.intState(0));
            SerialPrintln(usbTxBuf);
            printCounter = 0;
        }
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