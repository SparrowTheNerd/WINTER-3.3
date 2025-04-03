#ifndef __MS5607_H
#define __MS5607_H

#include "stm32h7xx_hal.h"

#define MS5607_ADDR 0x76<<1 // MS5607 I2C address
#define MS5607_REG_RESET 0x1E // Reset register
#define MS5607_REG_CONVERT_D1 0x40 // Convert D1 (pressure) register
#define MS5607_REG_CONVERT_D2 0x50 // Convert D2 (temperature) register
#define MS5607_REG_PROM_COEF 0xA2 // PROM coefficient register start
#define MS5607_REG_ADC_READ 0x00 // ADC read register

typedef struct {
    uint16_t C[6]; // Calibration coefficients
    uint32_t D1; // ADC value for pressure
    uint32_t D2; // ADC value for temperature
    int32_t dT; // Difference between actual and reference temperature
    int32_t TEMP; // Actual temperature
    int64_t OFF; // Offset at actual temperature
    int64_t SENS; // Sensitivity at actual temperature
    int32_t P; // Actual pressure
} MS5607_rawDat;

typedef struct {
    I2C_HandleTypeDef *hi2c; // I2C handle
    uint8_t OSR; // Oversampling rate (0-4 -> 256-4096)
    MS5607_rawDat raw;
    float temp; // Temperature in Celsius
    int32_t pres; // Pressure in Pa
    float alt; // Altitude in meters
    int available; // Data availability flag
} MS5607;


HAL_StatusTypeDef MS5607_Init(MS5607 *dev, I2C_HandleTypeDef *hi2c, uint8_t OSR);
HAL_StatusTypeDef MS5607_ReadRegs(MS5607 *dev, uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef MS5607_GetData(MS5607 *dev);
void MS5607_Convert(MS5607 *dev);


#endif /* __MS5607_H */