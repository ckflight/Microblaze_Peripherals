#ifndef ADXL362_H
#define ADXL362_H

#include "xspi.h"
#include <stdint.h>

// Public Functions
void ADXL362_Init(XSpi *SpiInstancePtr);
void ADXL362_SoftReset(XSpi *SpiInstancePtr);
uint8_t ADXL362_ReadDeviceID(XSpi *SpiInstancePtr);
void ADXL362_ReadXYZ(XSpi *SpiInstancePtr, int16_t *x, int16_t *y, int16_t *z, uint8_t *rx_buf);

#endif // ADXL362_H