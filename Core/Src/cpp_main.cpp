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

#include "fatfs.h"
#include "user_diskio.h"
#include <string.h>
#include <stdio.h>

#define USBBUF_MAXLEN 256


/* 
 * __attribute__((section(".nocache")));
 */

uint8_t usbTxBuf[USBBUF_MAXLEN];
uint8_t i2cBuf;
uint16_t usbTxBufLen;
int8_t highGOfst[3] = {0,0,0};
float imuOfst[3] = {0,0,0};

FATFS SDFatFS;              // The FATFS object

constexpr const char* TEST_FILE = "speed.bin";
constexpr size_t FILE_SIZE = 512 * 1024; // 512 KB
constexpr size_t BLOCK_SIZE = 512;
constexpr size_t BLOCK_FRAME_SIZE = BLOCK_SIZE + EXTRA_BYTES; // 512 bytes + 3 bytes for CRC and start byte
constexpr size_t dmaBlocks = 8;
constexpr size_t BUFFER_SIZE = BLOCK_FRAME_SIZE * dmaBlocks - (EXTRA_BYTES-3); // blocks plus CRC & start byte and end byte
constexpr size_t NUM_BLOCKS = FILE_SIZE / BLOCK_SIZE;

static uint8_t buffer[BUFFER_SIZE] __attribute__((section(".nocache"))); // Buffer for write operations
static uint8_t rxBuffer[BUFFER_SIZE] __attribute__((section(".nocache")));

/*
* Fill the actual data buffer with known pattern, and add start byte and dummy CRC bytes.
* The data pattern will not be interrupted by the start byte or CRC bytes because of the 
* use of ofst and dataIndex. This allows write and read integrity checks to be performed easily
*/ 
void fillBuffer(uint8_t* buf, size_t len, uint8_t seed) {
    uint32_t dataIndex = 0;
    for (size_t block = 0; block < dmaBlocks; block++) {
        size_t ofst = block * BLOCK_FRAME_SIZE;
        buf[ofst] = 0xFC; // Start byte

        for (uint32_t j = 0; j < BLOCK_SIZE; j++) {
            buf[ofst + 1 + j] = (dataIndex + seed) & 0xFF;
            dataIndex++;
        }
        if(block < dmaBlocks-1) {
            for(size_t j = 0; j < EXTRA_BYTES - 1; j++) {
                buf[ofst + BLOCK_SIZE + 1 + j] = 0xFF; // dummy CRC and padding
            }
        }
        else {
            buf[ofst + BLOCK_SIZE + 1] = 0xFF;
            buf[ofst + BLOCK_SIZE + 2] = 0xFF;
        }
    }
}

bool buffersMatch(const uint8_t* a, const uint8_t* b, size_t len) {
    return memcmp(a, b, len) == 0;
}

int cpp_main()
{
    // ADXL375 highG = ADXL375(&hi2c3, 10, highGOfst); // 10Hz ODR
    // MS5607 baro = MS5607(&hi2c3, 4); // 4096 OSR
    // ICM42688 imu = ICM42688(&hi2c3, imuOfst, imuOfst, _2000dps, _100Hz, _16g, _100Hz);  //full scale, 100hz

	// /* Initialization */
    // HAL_Delay(100);
    // HAL_StatusTypeDef status = highG.Init();
    // if (status != HAL_OK) {
    //     SerialPrintln((uint8_t*)"ADXL375 Init Failed!");
    // }

    // status = baro.Init(); // 4096 OSR
    // if (status != HAL_OK) {
    //     SerialPrintln((uint8_t*)"MS5607 Init Failed!");
    // }

    // status = imu.Init();
    // if (status != HAL_OK) {
    //     SerialPrintln((uint8_t*)"ICM42688 Init Failed!");
    // }



    HAL_Delay(5000);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    SerialPrintln((uint8_t*)"Mounting filesystem...");

    if (f_mount(&SDFatFS, "", 1) != FR_OK) {
        SerialPrintln((uint8_t*)"Failed to mount SD card.\r\n");
        while(1);
    }

    // Fill write buffer
    fillBuffer(buffer, BUFFER_SIZE, 0x00);

    // Write Test
    FIL file;
    FRESULT res;
    DSTATUS dres;
    if (f_open(&file, TEST_FILE, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        SerialPrintln((uint8_t*)"Failed to create file.");
        while(1);
    }

    f_expand(&file, FILE_SIZE,1); // Expand file to desired size
    f_lseek(&file, f_size(&file));
    // DWORD lba_start = 0;
    DWORD lba_start = ((file.obj.sclust - 2) * SDFatFS.csize) + SDFatFS.database;   //calculate the starting LBA

    SerialPrintln((uint8_t*)"Starting write benchmark...");

    uint32_t totalWriteStart = HAL_GetTick();
    uint32_t writeLatencySum = 0;
    UINT bytesWritten;
    for (size_t index = 0; index < FILE_SIZE/(dmaBlocks*BLOCK_SIZE); index++) {
        uint32_t t0 = DWT->CYCCNT;
        // res = f_write(&file, buffer, BLOCK_SIZE, &bytesWritten);
        dres = USER_DMA_Write(buffer, rxBuffer, BUFFER_SIZE, lba_start + index*dmaBlocks, dmaBlocks);
        uint32_t t1 = DWT->CYCCNT;

        if (dres != RES_OK) {
            SerialPrintln((uint8_t*)"Write error.");
            f_close(&file);
            while(1);
        }
        writeLatencySum += (t1 - t0);
    }
    uint32_t writeTimeMs = HAL_GetTick() - totalWriteStart;
    f_close(&file);

    // Read Test 
    if (f_open(&file, TEST_FILE, FA_READ) != FR_OK) {
        SerialPrintln((uint8_t*)"Failed to open file for reading.");
        while(1);
    }

    SerialPrintln((uint8_t*)"Starting read benchmark...");

    uint32_t totalReadStart = HAL_GetTick();
    uint32_t readLatencySum = 0;

    for (size_t offset = 0; offset < FILE_SIZE; offset += BLOCK_SIZE) {
        UINT bytesRead;
        uint32_t t0 = DWT->CYCCNT;
        res = f_read(&file, buffer, BLOCK_SIZE, &bytesRead);
        uint32_t t1 = DWT->CYCCNT;

        if (res != FR_OK || bytesRead != BLOCK_SIZE) {
            SerialPrintln((uint8_t*)"Read error.");
            f_close(&file);
            while(1);
        }
        readLatencySum += (t1 - t0);
    }
    uint32_t readTimeMs = HAL_GetTick() - totalReadStart;
    f_close(&file);

    // Calculate results
    float writeSpeedKBs = FILE_SIZE / 1024.0f / (writeTimeMs / 1000.0f);
    float readSpeedKBs  = FILE_SIZE / 1024.0f / (readTimeMs  / 1000.0f);

    // Convert to microseconds per block
    float cpuFreqMHz = HAL_RCC_GetHCLKFreq() / 1e6f;
    float avgWriteLatencyUs = (writeLatencySum / (float)NUM_BLOCKS / cpuFreqMHz)/dmaBlocks;
    float avgReadLatencyUs  = readLatencySum  / (float)NUM_BLOCKS / cpuFreqMHz;

    sprintf((char*)usbTxBuf, "FATFS Benchmark complete:\r\n"
             "Write: %.2f KB/s, Avg latency: %.2f µs/block\r\n"
             "Read:  %.2f KB/s, Avg latency: %.2f µs/block",
             writeSpeedKBs, avgWriteLatencyUs,
             readSpeedKBs, avgReadLatencyUs);
    usbTxBufLen = strlen((char*)usbTxBuf);
    SerialPrintln(usbTxBuf);

    f_unlink(TEST_FILE); // delete the test file


	while (1)
	{
        // // SerialPrintln((uint8_t*)"Hello world!");
        // for(uint8_t i=1; i<128; i++)
        // {
        //     uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(i<<1), 3, 5);
        //     if (ret != HAL_OK) /* No ACK Received At That Address */
        //     {
        //         continue;
        //     }
        //     else if(ret == HAL_OK)
        //     {
        //         sprintf((char*)usbTxBuf, "I2C Device Found: 0x%02X", i);
        //         usbTxBufLen = strlen((char*)usbTxBuf);
        //         SerialPrintln(usbTxBuf);
        //     }
        // }
        
        HAL_Delay(1000);
		
	}
}