#pragma once

#define ADDR_MAG 0x30
#define ADDR_GPS 0x42
#define USBBUF_MAXLEN 256


#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"

int cpp_main();

#ifdef __cplusplus
}
#endif