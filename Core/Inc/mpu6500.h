/*
 * mpu6500.h
 *
 *  Created on: Nov 24, 2025
 *      Author: user
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// MPU6500 SPI Settings
#define MPU6500_SPI_READ    0x80  // Read bit for SPI
#define MPU6500_SPI_WRITE   0x00  // Write bit for SPI

// MPU6500 Register Map
#define MPU6500_REG_WHO_AM_I        0x75
#define MPU6500_REG_PWR_MGMT_1      0x6B
#define MPU6500_REG_PWR_MGMT_2      0x6C
#define MPU6500_REG_SMPLRT_DIV      0x19
#define MPU6500_REG_CONFIG          0x1A
#define MPU6500_REG_GYRO_CONFIG     0x1B
#define MPU6500_REG_ACCEL_CONFIG    0x1C
#define MPU6500_REG_ACCEL_CONFIG_2  0x1D
#define MPU6500_REG_SIGNAL_PATH_RST 0x68
#define MPU6500_REG_USER_CTRL       0x6A

// Data Registers
#define MPU6500_REG_ACCEL_XOUT_H    0x3B
#define MPU6500_REG_ACCEL_XOUT_L    0x3C
#define MPU6500_REG_ACCEL_YOUT_H    0x3D
#define MPU6500_REG_ACCEL_YOUT_L    0x3E
#define MPU6500_REG_ACCEL_ZOUT_H    0x3F
#define MPU6500_REG_ACCEL_ZOUT_L    0x40
#define MPU6500_REG_TEMP_OUT_H      0x41
#define MPU6500_REG_TEMP_OUT_L      0x42
#define MPU6500_REG_GYRO_XOUT_H     0x43
#define MPU6500_REG_GYRO_XOUT_L     0x44
#define MPU6500_REG_GYRO_YOUT_H     0x45
#define MPU6500_REG_GYRO_YOUT_L     0x46
#define MPU6500_REG_GYRO_ZOUT_H     0x47
#define MPU6500_REG_GYRO_ZOUT_L     0x48

// Expected WHO_AM_I value
#define MPU6500_WHO_AM_I_VALUE      0x70

// Configuration register values
#define MPU6500_CLOCK_PLL_XGYRO     0x01
#define MPU6500_GYRO_FS_250_REG     0x00
#define MPU6500_GYRO_FS_500_REG     0x08
#define MPU6500_GYRO_FS_1000_REG    0x10
#define MPU6500_GYRO_FS_2000_REG    0x18
#define MPU6500_ACCEL_FS_2G_REG     0x00
#define MPU6500_ACCEL_FS_4G_REG     0x08
#define MPU6500_ACCEL_FS_8G_REG     0x10
#define MPU6500_ACCEL_FS_16G_REG    0x18

// Scale factors for conversion
#define MPU6500_ACCEL_SCALE_2G      16384.0f
#define MPU6500_ACCEL_SCALE_4G      8192.0f
#define MPU6500_ACCEL_SCALE_8G      4096.0f
#define MPU6500_ACCEL_SCALE_16G     2048.0f
#define MPU6500_GYRO_SCALE_250      131.0f
#define MPU6500_GYRO_SCALE_500      65.5f
#define MPU6500_GYRO_SCALE_1000     32.8f
#define MPU6500_GYRO_SCALE_2000     16.4f

//deg to rad
#define deg_to_rad 0.0174532925f

// Enums for configuration
typedef enum {
    MPU6500_GYRO_FS_250_DPS = 0,
    MPU6500_GYRO_FS_500_DPS,
    MPU6500_GYRO_FS_1000_DPS,
    MPU6500_GYRO_FS_2000_DPS
} MPU6500_GyroFS_t;

typedef enum {
    MPU6500_ACCEL_FS_2G = 0,
    MPU6500_ACCEL_FS_4G,
    MPU6500_ACCEL_FS_8G,
    MPU6500_ACCEL_FS_16G
} MPU6500_AccelFS_t;

// Structure to hold MPU6500 data
typedef struct {
    // Raw data
    int16_t accel_raw[3];     // X, Y, Z
    int16_t gyro_raw[3];      // X, Y, Z
    int16_t temp_raw;

    // Processed data
    float accel_g[3];         // Acceleration in g
    float gyro_dps[3];        // Angular velocity in degrees/second
    float gyro_rad[3];
    float temp_c;             // Temperature in Celsius

    //offset
    float gyro_bias[3];

    // Configuration
    MPU6500_GyroFS_t gyro_fs;
    MPU6500_AccelFS_t accel_fs;
    float gyro_scale;
    float accel_scale;

    // SPI Communication
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    bool initialized;
    // DMA support
    uint8_t dma_tx_buffer[15];     // TX buffer for DMA (1 addr + 14 dummy)
    uint8_t dma_rx_buffer[15];     // RX buffer for DMA (14 data bytes)
    volatile bool dma_transfer_complete;
} MPU6500_t;

// Function prototypes
HAL_StatusTypeDef MPU6500_Init(MPU6500_t *mpu, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
HAL_StatusTypeDef MPU6500_SetGyroFS(MPU6500_t *mpu, MPU6500_GyroFS_t fs);
HAL_StatusTypeDef MPU6500_SetAccelFS(MPU6500_t *mpu, MPU6500_AccelFS_t fs);
HAL_StatusTypeDef MPU6500_ReadRaw(MPU6500_t *mpu);
HAL_StatusTypeDef MPU6500_CalibrateGyro(MPU6500_t *mpu, uint16_t num_samples);
HAL_StatusTypeDef MPU6500_ReadProcessed(MPU6500_t *mpu);
HAL_StatusTypeDef MPU6500_ReadAccelRaw(MPU6500_t *mpu);
HAL_StatusTypeDef MPU6500_ReadGyroRaw(MPU6500_t *mpu);
HAL_StatusTypeDef MPU6500_ReadTempRaw(MPU6500_t *mpu);

// Utility functions
HAL_StatusTypeDef MPU6500_WriteRegister(MPU6500_t *mpu, uint8_t reg, uint8_t data);
HAL_StatusTypeDef MPU6500_ReadRegister(MPU6500_t *mpu, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MPU6500_ReadRegisters(MPU6500_t *mpu, uint8_t reg, uint8_t *data, uint8_t length);
bool MPU6500_IsConnected(MPU6500_t *mpu);

//DMA functions
HAL_StatusTypeDef MPU6500_ReadRaw_DMA(MPU6500_t *mpu);
void MPU6500_DMA_Complete_Callback(MPU6500_t *mpu);


#endif /* INC_MPU6500_H_ */
