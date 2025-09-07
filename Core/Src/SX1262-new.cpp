#include "SX1262-new.h"

#define CS_HIGH()	{HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);}
#define CS_LOW()	{HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);}

SX1262::SX1262(SPI_HandleTypeDef *hspi, uint32_t freq) {
    this->hspi = hspi;
    this->freq = freq;
}

HAL_StatusTypeDef SX1262::SendBytes(uint8_t opcode, uint8_t* dat, uint8_t len) {
    uint8_t txDat[len+1] = {0};
    txDat[0] = opcode;
    for(int i=0;i<len;i++) { txDat[i+1] = dat[i]; }    //create a byte array of the opcode and data bytes
    CS_LOW();
    HAL_SPI_Transmit(hspi,txDat,len+1,HAL_MAX_DELAY);
    CS_HIGH();
}

HAL_StatusTypeDef SX1262::Reset() {
    CS_HIGH();
    HAL_GPIO_WritePin(LoRa_RST_GPIO_Port,LoRa_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LoRa_RST_GPIO_Port,LoRa_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

HAL_StatusTypeDef SX1262::SetFrequency(uint32_t frequency) {
    uint8_t buf[4];

    uint32_t freq = (uint32_t)((double)frequency / ((double)32e6 / pow(2,25)));
    buf[0] = ((freq >> 24) & 0xFF);
    buf[1] = ((freq >> 16) & 0xFF);
    buf[2] = ((freq >> 8) & 0xFF);
    buf[3] = (freq & 0xFF);

    return SendBytes(SX1262_CMD_SET_RF_FREQUENCY, buf, 4);
}

HAL_StatusTypeDef SX1262::Init() {
    uint8_t res;
    res = Reset();
    res |= SendBytes(SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, (uint8_t*)SX1262_DIO3_OUTPUT_2_4, 1);   // Set DIO3 to 2.4V TCXO Control
    res |= SetFrequency(freq);


    return (HAL_StatusTypeDef)res;
}