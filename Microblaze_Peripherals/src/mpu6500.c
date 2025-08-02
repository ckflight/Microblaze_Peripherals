#include "mpu6500.h"
#include <unistd.h>
#include <stdint.h>

#define READ_CMD         0x80
#define WHO_AM_I_REG     0x75
#define PWR_MGMT_1       0x6B
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43
#define WHO_AM_I_EXPECTED 0x70

static void mpu6500_write_reg(XSpi *spi, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg & 0x7F, value };
    XSpi_SetSlaveSelect(spi, 1);
    XSpi_Transfer(spi, tx, NULL, 2);
    usleep(1000);
}

static uint8_t mpu6500_read_reg(XSpi *spi, uint8_t reg) {
    uint8_t tx[2] = { reg | READ_CMD, 0xFF };
    uint8_t rx[2] = {0};
    XSpi_SetSlaveSelect(spi, 1);
    XSpi_Transfer(spi, tx, rx, 2);
    return rx[1];
}

uint8_t MPU6500_Init(XSpi *spi) {
    mpu6500_write_reg(spi, PWR_MGMT_1, 0x80);
    usleep(100000);
    mpu6500_write_reg(spi, PWR_MGMT_1, 0x01);
    mpu6500_write_reg(spi, GYRO_CONFIG,  0x18); // ±2000 dps
    mpu6500_write_reg(spi, ACCEL_CONFIG, 0x18); // ±16g
    return mpu6500_read_reg(spi, WHO_AM_I_REG);
}

void MPU6500_ReadAccel(XSpi *spi, int16_t *accel) {
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};

    tx[0] = ACCEL_XOUT_H | READ_CMD;
    XSpi_SetSlaveSelect(spi, 1);
    XSpi_Transfer(spi, tx, rx, 7);

    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((rx[1 + i * 2] << 8) | rx[2 + i * 2]);
    }
}

void MPU6500_ReadGyro(XSpi *spi, int16_t *gyro) {
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};

    tx[0] = GYRO_XOUT_H | READ_CMD;
    XSpi_SetSlaveSelect(spi, 1);
    XSpi_Transfer(spi, tx, rx, 7);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (int16_t)((rx[1 + i * 2] << 8) | rx[2 + i * 2]);
    }
}

void MPU6500_ConvertGyroToDPS(const int16_t *raw, float *dps_out) {
    for (int i = 0; i < 3; i++) {
        dps_out[i] = (float)raw[i] / 16.4f;  // ±2000 dps range
    }
}