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

MMC5983 mag(&hi2c3, MMC5983::_400Hz, MMC5983::_200hz, MMC5983::_75);
ICM42688 imu(&hi2c3, (double[3]){-0.007883,-0.034859,0.199367}, (double[3]){0,0,0}, ICM42688::_1000dps, ICM42688::_1kHz, ICM42688::_16g, ICM42688::_1kHz);
MS5607 baro(&hi2c3, 0);
ADXL375 highG(&hi2c3, 0b1100, (int8_t[3]){0,0,0});
SFE_UBLOX_GNSS myGNSS;

KalmanFilter ekf;


uint16_t bufLen = 0;

void print_matrix(Eigen::MatrixXd X);

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
    while(baro.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"Barometer Init Failed"); HAL_Delay(1000);
    }
    HAL_Delay(2000);

    ekf.init(&mag, &imu, &baro, &highG, &myGNSS);
    prevTime = (double)(HAL_GetTick())/1000.0;
    uint8_t printCounter = 0;

    // Enable DWT Cycle Counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55; 
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t start = DWT->CYCCNT; // Get current cycle count
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * 10000; // 10.000 ms of cycles
    // SerialPrintln((uint8_t*)"ICMgX, ICMgY, ICMgZ, ICMaX, ICMaY, ICMaZ, ADXLx, ADXLy, ADXLz, MagX, MagY, MagZ, Baro");

	while (1)
	{   
        
        // cycles = (freq / 1000000(uS/S)) * uS
        // uS = cycles / (freq / 1000000(uS/S))
        // S = cycles / freq
        if(DWT->CYCCNT - start >= cycles) {
            dt = (double)(DWT->CYCCNT - start) / (double)HAL_RCC_GetHCLKFreq(); // Calculate elapsed time in s
            start = DWT->CYCCNT; // Reset start time
            // dt = (double)(HAL_GetTick())/1000.0 - prevTime;
            ekf.predict(dt);
            ekf.update();
            // printCounter++;
            // HAL_Delay(10);
            
            // if(printCounter > 5) {
                print_matrix(ekf.intState.transpose());
                // HAL_Delay(2);
                // print_matrix(ekf.errState.transpose());
                // sprintf((char*)usbTxBuf,"Pressure: %d Pa",baro.pres);
                // SerialPrintln(usbTxBuf);

                // HAL_Delay(1);
                // sprintf((char*)usbTxBuf, "%.5f \t %.5f \t %.5f    ",imu.accel_ms2[0], imu.accel_ms2[1], imu.accel_ms2[2]);
                // sprintf((char*)usbTxBuf, ">3D|Orientation:S:cube:P:0:0:0:Q:%.5f:%.5f:%.5f:%.5f:W:1:H:1:D:1:C:#ff0000", ekf.intState(1), ekf.intState(2), ekf.intState(3), ekf.intState(0));
                // sprintf((char*)usbTxBuf, ">aX: %.5f\r\n>aY: %.5f\r\n>aZ: %.5f", imu.accel_ms2[0], imu.accel_ms2[1], imu.accel_ms2[2]);
                // SerialPrintln(usbTxBuf);
                // printCounter = 0;
            // }
        }
		// HAL_Delay(5);
	}
}

void print_matrix(Eigen::MatrixXd X)  
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

/*
imu.ReadIMU();
mag.ReadMag();
highG.ReadAccel();
baro.GetData();
if(baro.available) {
    baro.Convert();
    sprintf((char*)usbTxBuf,"%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", \
        imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2], imu.accel_ms2[0], imu.accel_ms2[1], imu.accel_ms2[2], \
        highG.accel_ms2[0], highG.accel_ms2[1], highG.accel_ms2[2], \
        mag.mag_gauss.x(), mag.mag_gauss.y(), mag.mag_gauss.z(), \
        baro.alt);
}
else {
    sprintf((char*)usbTxBuf,"%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f,", \
        imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2], imu.accel_ms2[0], imu.accel_ms2[1], imu.accel_ms2[2], \
        highG.accel_ms2[0], highG.accel_ms2[1], highG.accel_ms2[2], \
        mag.mag_gauss.x(), mag.mag_gauss.y(), mag.mag_gauss.z());
}
SerialPrintln(usbTxBuf);
*/