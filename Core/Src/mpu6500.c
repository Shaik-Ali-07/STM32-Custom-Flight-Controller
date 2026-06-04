/*
 * mpu6500.c
 *
 *  Created on: Nov 24, 2025
 *      Author: user
 */



#include "mpu6500.h"

// CS Pin Control Macros
#define MPU6500_CS_LOW()   HAL_GPIO_WritePin(mpu->cs_port, mpu->cs_pin, GPIO_PIN_RESET)
#define MPU6500_CS_HIGH()  HAL_GPIO_WritePin(mpu->cs_port, mpu->cs_pin, GPIO_PIN_SET)

// Private function to set scale factors
static void MPU6500_SetScaleFactors(MPU6500_t *mpu) {
    // Set gyroscope scale factor
    switch (mpu->gyro_fs) {
        case MPU6500_GYRO_FS_250_DPS:
            mpu->gyro_scale = MPU6500_GYRO_SCALE_250;
            break;
        case MPU6500_GYRO_FS_500_DPS:
            mpu->gyro_scale = MPU6500_GYRO_SCALE_500;
            break;
        case MPU6500_GYRO_FS_1000_DPS:
            mpu->gyro_scale = MPU6500_GYRO_SCALE_1000;
            break;
        case MPU6500_GYRO_FS_2000_DPS:
            mpu->gyro_scale = MPU6500_GYRO_SCALE_2000;
            break;
    }

    // Set accelerometer scale factor
    switch (mpu->accel_fs) {
        case MPU6500_ACCEL_FS_2G:
            mpu->accel_scale = MPU6500_ACCEL_SCALE_2G;
            break;
        case MPU6500_ACCEL_FS_4G:
            mpu->accel_scale = MPU6500_ACCEL_SCALE_4G;
            break;
        case MPU6500_ACCEL_FS_8G:
            mpu->accel_scale = MPU6500_ACCEL_SCALE_8G;
            break;
        case MPU6500_ACCEL_FS_16G:
            mpu->accel_scale = MPU6500_ACCEL_SCALE_16G;
            break;
    }
}

// Write to MPU6500 register via SPI
HAL_StatusTypeDef MPU6500_WriteRegister(MPU6500_t *mpu, uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];

    tx_data[0] = reg & 0x7F;  // Clear MSB for write operation
    tx_data[1] = data;

    MPU6500_CS_LOW();
    status = HAL_SPI_Transmit(mpu->hspi, tx_data, 2, HAL_MAX_DELAY);
    MPU6500_CS_HIGH();

    return status;
}

// Read from MPU6500 register via SPI
HAL_StatusTypeDef MPU6500_ReadRegister(MPU6500_t *mpu, uint8_t reg, uint8_t *data) {
    HAL_StatusTypeDef status;
    uint8_t tx_data;

    tx_data = reg | MPU6500_SPI_READ;  // Set MSB for read operation

    MPU6500_CS_LOW();
    status = HAL_SPI_Transmit(mpu->hspi, &tx_data, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(mpu->hspi, data, 1, HAL_MAX_DELAY);
    }
    MPU6500_CS_HIGH();

    return status;
}

// Read multiple registers from MPU6500 via SPI
HAL_StatusTypeDef MPU6500_ReadRegisters(MPU6500_t *mpu, uint8_t reg, uint8_t *data, uint8_t length) {
    HAL_StatusTypeDef status;
    uint8_t tx_data;

    tx_data = reg | MPU6500_SPI_READ;  // Set MSB for read operation

    MPU6500_CS_LOW();
    status = HAL_SPI_Transmit(mpu->hspi, &tx_data, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(mpu->hspi, data, length, HAL_MAX_DELAY);
    }
    MPU6500_CS_HIGH();

    return status;
}

// Check if MPU6500 is connected
bool MPU6500_IsConnected(MPU6500_t *mpu) {
    uint8_t who_am_i;
    if (MPU6500_ReadRegister(mpu, MPU6500_REG_WHO_AM_I, &who_am_i) == HAL_OK) {
        return (who_am_i == MPU6500_WHO_AM_I_VALUE);
    }
    return false;
}

// Initialize MPU6500 with SPI
HAL_StatusTypeDef MPU6500_Init(MPU6500_t *mpu, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    HAL_StatusTypeDef status;

    // Set SPI handle and CS pin
    mpu->hspi = hspi;
    mpu->cs_port = cs_port;
    mpu->cs_pin = cs_pin;
    mpu->initialized = false;

    // Initialize CS pin to HIGH (inactive)
    MPU6500_CS_HIGH();
    HAL_Delay(10);

    // Check WHO_AM_I register
    if (!MPU6500_IsConnected(mpu)) {
        return HAL_ERROR;
    }

    // Reset the device
    status = MPU6500_WriteRegister(mpu, MPU6500_REG_PWR_MGMT_1, 0x80);
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    // Disable I2C interface (important for SPI mode)
    status = MPU6500_WriteRegister(mpu, MPU6500_REG_USER_CTRL, 0x10);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Wake up device and set clock source
    status = MPU6500_WriteRegister(mpu, MPU6500_REG_PWR_MGMT_1, MPU6500_CLOCK_PLL_XGYRO);
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    // Enable all sensors
    status = MPU6500_WriteRegister(mpu, MPU6500_REG_PWR_MGMT_2, 0x00);
    if (status != HAL_OK) return status;

    // Set sample rate divider (1kHz / (1 + 7) = 125Hz)
    status = MPU6500_WriteRegister(mpu, MPU6500_REG_SMPLRT_DIV, 0x13);
    if (status != HAL_OK) return status;

    // Configure DLPF (Digital Low Pass Filter)
    status = MPU6500_WriteRegister(mpu, MPU6500_REG_CONFIG, 0x04);
    if (status != HAL_OK) return status;

    // Set default full scale ranges
    mpu->gyro_fs = MPU6500_GYRO_FS_500_DPS;
    mpu->accel_fs = MPU6500_ACCEL_FS_4G;

    // Configure gyroscope
    status = MPU6500_SetGyroFS(mpu, mpu->gyro_fs);
    if (status != HAL_OK) return status;

    // Configure accelerometer
    status = MPU6500_SetAccelFS(mpu, mpu->accel_fs);
    if (status != HAL_OK) return status;

    // Set accelerometer DLPF
    status = MPU6500_WriteRegister(mpu, MPU6500_REG_ACCEL_CONFIG_2, 0x06);
    if (status != HAL_OK) return status;

    // Set scale factors
    MPU6500_SetScaleFactors(mpu);

    mpu->initialized = true;
    return HAL_OK;
}

// Set gyroscope full scale range
HAL_StatusTypeDef MPU6500_SetGyroFS(MPU6500_t *mpu, MPU6500_GyroFS_t fs) {
    HAL_StatusTypeDef status;
    uint8_t config_value;

    switch (fs) {
        case MPU6500_GYRO_FS_250_DPS:
            config_value = MPU6500_GYRO_FS_250_REG;
            break;
        case MPU6500_GYRO_FS_500_DPS:
            config_value = MPU6500_GYRO_FS_500_REG;
            break;
        case MPU6500_GYRO_FS_1000_DPS:
            config_value = MPU6500_GYRO_FS_1000_REG;
            break;
        case MPU6500_GYRO_FS_2000_DPS:
            config_value = MPU6500_GYRO_FS_2000_REG;
            break;
        default:
            return HAL_ERROR;
    }

    status = MPU6500_WriteRegister(mpu, MPU6500_REG_GYRO_CONFIG, config_value);
    if (status == HAL_OK) {
        mpu->gyro_fs = fs;
        MPU6500_SetScaleFactors(mpu);
    }

    return status;
}

// Set accelerometer full scale range
HAL_StatusTypeDef MPU6500_SetAccelFS(MPU6500_t *mpu, MPU6500_AccelFS_t fs) {
    HAL_StatusTypeDef status;
    uint8_t config_value;

    switch (fs) {
        case MPU6500_ACCEL_FS_2G:
            config_value = MPU6500_ACCEL_FS_2G_REG;
            break;
        case MPU6500_ACCEL_FS_4G:
            config_value = MPU6500_ACCEL_FS_4G_REG;
            break;
        case MPU6500_ACCEL_FS_8G:
            config_value = MPU6500_ACCEL_FS_8G_REG;
            break;
        case MPU6500_ACCEL_FS_16G:
            config_value = MPU6500_ACCEL_FS_16G_REG;
            break;
        default:
            return HAL_ERROR;
    }

    status = MPU6500_WriteRegister(mpu, MPU6500_REG_ACCEL_CONFIG, config_value);
    if (status == HAL_OK) {
        mpu->accel_fs = fs;
        MPU6500_SetScaleFactors(mpu);
    }

    return status;
}

// Read raw accelerometer data
HAL_StatusTypeDef MPU6500_ReadAccelRaw(MPU6500_t *mpu) {
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = MPU6500_ReadRegisters(mpu, MPU6500_REG_ACCEL_XOUT_H, data, 6);
    if (status == HAL_OK) {
        mpu->accel_raw[0] = (int16_t)((data[0] << 8) | data[1]); // X
        mpu->accel_raw[1] = (int16_t)((data[2] << 8) | data[3]); // Y
        mpu->accel_raw[2] = (int16_t)((data[4] << 8) | data[5]); // Z
    }

    return status;
}

// Read raw gyroscope data
HAL_StatusTypeDef MPU6500_ReadGyroRaw(MPU6500_t *mpu) {
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = MPU6500_ReadRegisters(mpu, MPU6500_REG_GYRO_XOUT_H, data, 6);
    if (status == HAL_OK) {
        mpu->gyro_raw[0] = (int16_t)((data[0] << 8) | data[1]); // X
        mpu->gyro_raw[1] = (int16_t)((data[2] << 8) | data[3]); // Y
        mpu->gyro_raw[2] = (int16_t)((data[4] << 8) | data[5]); // Z
    }

    return status;
}

// Read raw temperature data
HAL_StatusTypeDef MPU6500_ReadTempRaw(MPU6500_t *mpu) {
    uint8_t data[2];
    HAL_StatusTypeDef status;

    status = MPU6500_ReadRegisters(mpu, MPU6500_REG_TEMP_OUT_H, data, 2);
    if (status == HAL_OK) {
        mpu->temp_raw = (int16_t)((data[0] << 8) | data[1]);
    }

    return status;
}

// Read all raw sensor data
HAL_StatusTypeDef MPU6500_ReadRaw(MPU6500_t *mpu) {
    uint8_t data[14];
    HAL_StatusTypeDef status;

    // Read all sensor data in one transaction (more efficient)
    status = MPU6500_ReadRegisters(mpu, MPU6500_REG_ACCEL_XOUT_H, data, 14);
    if (status == HAL_OK) {
        // Parse accelerometer data
        mpu->accel_raw[0] = (int16_t)((data[0] << 8) | data[1]);   // ACCEL_X
        mpu->accel_raw[1] = (int16_t)((data[2] << 8) | data[3]);   // ACCEL_Y
        mpu->accel_raw[2] = (int16_t)((data[4] << 8) | data[5]);   // ACCEL_Z

        // Parse temperature data
        mpu->temp_raw = (int16_t)((data[6] << 8) | data[7]);       // TEMP

        // Parse gyroscope data
        mpu->gyro_raw[0] = (int16_t)((data[8] << 8) | data[9]);    // GYRO_X
        mpu->gyro_raw[1] = (int16_t)((data[10] << 8) | data[11]);  // GYRO_Y
        mpu->gyro_raw[2] = (int16_t)((data[12] << 8) | data[13]);  // GYRO_Z
    }

    return status;
}

// ADD THIS FUNCTION: Calibrate Gyroscope
HAL_StatusTypeDef MPU6500_CalibrateGyro(MPU6500_t *mpu, uint16_t num_samples) {
    if (num_samples == 0) return HAL_ERROR;

    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    HAL_StatusTypeDef status;

    // 1. Read N samples
    for (uint16_t i = 0; i < num_samples; i++) {
        status = MPU6500_ReadGyroRaw(mpu);
        if (status != HAL_OK) return status;

        // Convert to DPS immediately and accumulate
        gyro_sum[0] += (float)mpu->gyro_raw[0] / mpu->gyro_scale;
        gyro_sum[1] += (float)mpu->gyro_raw[1] / mpu->gyro_scale;
        gyro_sum[2] += (float)mpu->gyro_raw[2] / mpu->gyro_scale;

        HAL_Delay(3); // Small delay between samples
    }

    // 2. Calculate Average (Bias)
    mpu->gyro_bias[0] = gyro_sum[0] / num_samples;
    mpu->gyro_bias[1] = gyro_sum[1] / num_samples;
    mpu->gyro_bias[2] = gyro_sum[2] / num_samples;

    return HAL_OK;
}

// Read and process all sensor data
HAL_StatusTypeDef MPU6500_ReadProcessed(MPU6500_t *mpu) {
    HAL_StatusTypeDef status;

    // Read raw data first
    status = MPU6500_ReadRaw(mpu);
    if (status != HAL_OK) return status;

    // Convert accelerometer data to g
    mpu->accel_g[0] = (float)mpu->accel_raw[0] / mpu->accel_scale;
    mpu->accel_g[1] = (float)mpu->accel_raw[1] / mpu->accel_scale;
    mpu->accel_g[2] = (float)mpu->accel_raw[2] / mpu->accel_scale;

    // Convert gyroscope data to degrees/second AND SUBTRACT BIAS
    mpu->gyro_dps[0] = ((float)mpu->gyro_raw[0] / mpu->gyro_scale) - mpu->gyro_bias[0];
    mpu->gyro_dps[1] = ((float)mpu->gyro_raw[1] / mpu->gyro_scale) - mpu->gyro_bias[1];
    mpu->gyro_dps[2] = ((float)mpu->gyro_raw[2] / mpu->gyro_scale) - mpu->gyro_bias[2];

    //gyroscope value in rad
    mpu->gyro_rad[0] = mpu->gyro_dps[0] * deg_to_rad;
    mpu->gyro_rad[1] = mpu->gyro_dps[1] * deg_to_rad;
    mpu->gyro_rad[2] = mpu->gyro_dps[2] * deg_to_rad;

    // Convert temperature to Celsius
    mpu->temp_c = ((float)mpu->temp_raw / 333.87f) + 21.0f;

    return HAL_OK;
}

/**
 * @brief  Read all sensor data using DMA (non-blocking)
 * @param  mpu: Pointer to MPU6500_t structure
 * @retval HAL_OK if DMA transfer started successfully
 */
HAL_StatusTypeDef MPU6500_ReadRaw_DMA(MPU6500_t *mpu) {
    // Prepare TX buffer: first byte is register address with read bit
    mpu->dma_tx_buffer[0] = MPU6500_REG_ACCEL_XOUT_H | MPU6500_SPI_READ;
    // Rest are dummy bytes (0x00 or 0xFF - doesn't matter)
    for(int i = 1; i < 15; i++) {
        mpu->dma_tx_buffer[i] = 0x00;
    }

    // 2. Start CS LOW
        MPU6500_CS_LOW();

        // 3. Mark incomplete
        mpu->dma_transfer_complete = false;

        // 4. Try to start DMA
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(mpu->hspi,
                                           mpu->dma_tx_buffer,
                                           mpu->dma_rx_buffer,
                                           15);

        // 5. ERROR HANDLING (Crucial)
        if (status != HAL_OK) {
            // If DMA failed to start, release CS and reset flag so we can try next tick
            MPU6500_CS_HIGH();
            mpu->dma_transfer_complete = true;
        }

        return status;
}

/**
 * @brief  Process data after DMA transfer completes
 * @param  mpu: Pointer to MPU6500_t structure
 * @retval None
 */
void MPU6500_DMA_Complete_Callback(MPU6500_t *mpu) {
    // Parse received data (first byte is garbage, data starts at index 1)
    uint8_t *data = &mpu->dma_rx_buffer[1];

    // Parse accelerometer data
    mpu->accel_raw[0] = (int16_t)((data[0] << 8) | data[1]);   // ACCEL_X
    mpu->accel_raw[1] = (int16_t)((data[2] << 8) | data[3]);   // ACCEL_Y
    mpu->accel_raw[2] = (int16_t)((data[4] << 8) | data[5]);   // ACCEL_Z

    // Parse temperature data
    mpu->temp_raw = (int16_t)((data[6] << 8) | data[7]);       // TEMP

    // Parse gyroscope data
    mpu->gyro_raw[0] = (int16_t)((data[8] << 8) | data[9]);    // GYRO_X
    mpu->gyro_raw[1] = (int16_t)((data[10] << 8) | data[11]);  // GYRO_Y
    mpu->gyro_raw[2] = (int16_t)((data[12] << 8) | data[13]);  // GYRO_Z

    // Mark transfer complete
    mpu->dma_transfer_complete = true;
}
