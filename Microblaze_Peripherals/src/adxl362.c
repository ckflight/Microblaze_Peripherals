#include "adxl362.h"
#include <unistd.h> // for usleep

void ADXL362_SoftReset(XSpi *SpiInstancePtr)
{
    uint8_t reset_cmd[3] = {0x0A, 0x1F, 0x52}; // Write, Reset Register, Reset Code
    XSpi_SetSlaveSelect(SpiInstancePtr, 0x01);
    XSpi_Transfer(SpiInstancePtr, reset_cmd, NULL, 3);
    usleep(10000); // Wait 10ms
}

void ADXL362_Init(XSpi *SpiInstancePtr)
{
    uint8_t config1[] = {0x0A, 0x2D, 0x00}; // Standby
    uint8_t config2[] = {0x0A, 0x0E, 0x03}; // ODR = 100Hz
    uint8_t config3[] = {0x0A, 0x2D, 0x02}; // Measure mode

    XSpi_SetSlaveSelect(SpiInstancePtr, 0x01);
    XSpi_Transfer(SpiInstancePtr, config1, NULL, 3);
    usleep(1000);

    XSpi_SetSlaveSelect(SpiInstancePtr, 0x01);
    XSpi_Transfer(SpiInstancePtr, config2, NULL, 3);
    usleep(1000);

    XSpi_SetSlaveSelect(SpiInstancePtr, 0x01);
    XSpi_Transfer(SpiInstancePtr, config3, NULL, 3);
    usleep(1000);
}

uint8_t ADXL362_ReadDeviceID(XSpi *SpiInstancePtr)
{
    uint8_t tx[3] = {0x0B, 0x02, 0xFF};
    uint8_t rx[3] = {0};

    XSpi_SetSlaveSelect(SpiInstancePtr, 1);
    XSpi_Transfer(SpiInstancePtr, tx, rx, 3);

    return rx[2];
}

void ADXL362_ReadXYZ(XSpi *SpiInstancePtr, int16_t *x, int16_t *y, int16_t *z, uint8_t *rx_buf)
{
    uint8_t tx[7] = {0x0B, 0x0E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx[7] = {0};

    XSpi_SetSlaveSelect(SpiInstancePtr, 0x01);
    XSpi_Transfer(SpiInstancePtr, tx, rx, 7);

    memcpy(rx_buf, rx, sizeof(rx));  // Copies all 9 bytes from rx[] to rx_buf[]

    *x = (int16_t)((rx[3] << 8) | rx[2]);
    *y = (int16_t)((rx[5] << 8) | rx[4]);
    *z = (int16_t)((rx[7] << 8) | rx[6]);
}