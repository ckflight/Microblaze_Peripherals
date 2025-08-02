#ifndef MPU6500_H
#define MPU6500_H

#include "xspi.h"
#include <stdint.h>

uint8_t MPU6500_Init(XSpi *spi);

void MPU6500_ReadAccel(XSpi *spi, int16_t *accel);
void MPU6500_ReadGyro(XSpi *spi, int16_t *gyro);
void MPU6500_ConvertGyroToDPS(const int16_t *raw, float *dps_out);

#endif // MPU6500_H