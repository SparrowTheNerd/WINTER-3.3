#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t SerialPrint(uint8_t txBuf[]);
uint8_t SerialPrintln(uint8_t txBuf[]);

uint8_t i2cRead(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

uint32_t analogReadSE(uint32_t channel);

void delay_us(uint16_t us);
#ifdef __cplusplus
}
#endif