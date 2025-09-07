#pragma once

#include "stm32h7xx_hal.h"
#include "SX1262_Definitions.h"

class SX1262 {
    public:
        SX1262(SPI_HandleTypeDef *hspi, uint32_t freq);

        HAL_StatusTypeDef Init();
        HAL_StatusTypeDef Transmit(uint8_t* data, uint8_t len);
        HAL_StatusTypeDef SetRX(void);
        HAL_StatusTypeDef setModeStandby(void);
        HAL_StatusTypeDef setModeReceive(void);

    private:
        SPI_HandleTypeDef *hspi;
        uint32_t freq;
        HAL_StatusTypeDef SendBytes(uint8_t opcode, uint8_t* dat, uint8_t len);
        HAL_StatusTypeDef Reset();
        HAL_StatusTypeDef SetFrequency(uint32_t frequency);
};