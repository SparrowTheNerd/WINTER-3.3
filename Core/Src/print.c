#include "print.h"
#include "usbd_cdc_if.h"

//print a string to the USB serial port
uint8_t SerialPrint(char txBuf[]) {
    uint16_t bufLen = sizeof(txBuf);
    return CDC_Transmit_HS(txBuf, bufLen);
}

//print a string to the USB serial port with a newline
uint8_t SerialPrintln(char txBuf[]) {
    uint16_t bufLen = sizeof(txBuf);
    CDC_Transmit_HS(txBuf, bufLen);
    return CDC_Transmit_HS("\r\n", 2);
}