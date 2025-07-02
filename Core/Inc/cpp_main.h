#ifndef CPP_MAIN_H_
#define CPP_MAIN_H_

#define ADDR_MAG 0x30
#define ADDR_GPS 0x42

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"

int cpp_main();

#ifdef __cplusplus
}
#endif

#endif