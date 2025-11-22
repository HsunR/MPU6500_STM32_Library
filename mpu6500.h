/**
 * @file mpu6500.h 
 * @brief MPU6500 accelerometer and gyroscope driver header
 * @details This header file contains the declarations and definitions for the
 *          MPU6500 accelerometer and gyroscope driver, supporting various
 *          operations.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.0
 * @date 2025-06-08
 */

#ifndef __MPU6500_H__
#define __MPU6500_H__

#ifdef __cplusplus
extern "C" {
#endif

/* For HAL functions and pin definitions */
#include "main.h"

/* 陀螺仪满量程选择配置常量 */
#define MPU6500_GYRO_FS_250DPS     0x00  // ±250°/s
#define MPU6500_GYRO_FS_500DPS     0x08  // ±500°/s
#define MPU6500_GYRO_FS_1000DPS    0x10  // ±1000°/s
#define MPU6500_GYRO_FS_2000DPS    0x18  // ±2000°/s

/* 加速度计满量程选择配置常量 */
#define MPU6500_ACCEL_FS_2G        0x00  // ±2g
#define MPU6500_ACCEL_FS_4G        0x08  // ±4g
#define MPU6500_ACCEL_FS_8G        0x10  // ±8g
#define MPU6500_ACCEL_FS_16G       0x18  // ±16g

/* 陀螺仪灵敏度转换因子（LSB/°/s） */
#define MPU6500_GYRO_SENS_250DPS   131.0f
#define MPU6500_GYRO_SENS_500DPS   65.5f
#define MPU6500_GYRO_SENS_1000DPS  32.8f
#define MPU6500_GYRO_SENS_2000DPS  16.4f

/* 加速度计灵敏度转换因子（LSB/g） */
#define MPU6500_ACCEL_SENS_2G      16384.0f
#define MPU6500_ACCEL_SENS_4G      8192.0f
#define MPU6500_ACCEL_SENS_8G      4096.0f
#define MPU6500_ACCEL_SENS_16G     2048.0f

/* 默认配置设置 */
#define MPU6500_DEFAULT_ACCEL_CONFIG  MPU6500_ACCEL_FS_4G        // 默认加速度计量程：±4g
#define MPU6500_DEFAULT_GYRO_CONFIG   MPU6500_GYRO_FS_500DPS     // 默认陀螺仪量程：±500°/s

/* 根据默认陀螺仪配置动态选择灵敏度 */
#if MPU6500_DEFAULT_GYRO_CONFIG == MPU6500_GYRO_FS_250DPS
  #define MPU6500_GYRO_SENS  MPU6500_GYRO_SENS_250DPS
#elif MPU6500_DEFAULT_GYRO_CONFIG == MPU6500_GYRO_FS_500DPS
  #define MPU6500_GYRO_SENS  MPU6500_GYRO_SENS_500DPS
#elif MPU6500_DEFAULT_GYRO_CONFIG == MPU6500_GYRO_FS_1000DPS
  #define MPU6500_GYRO_SENS  MPU6500_GYRO_SENS_1000DPS
#elif MPU6500_DEFAULT_GYRO_CONFIG == MPU6500_GYRO_FS_2000DPS
  #define MPU6500_GYRO_SENS  MPU6500_GYRO_SENS_2000DPS
#else
  #error "Invalid gyroscope configuration"
#endif

/* 根据默认加速度计配置动态选择灵敏度 */
#if MPU6500_DEFAULT_ACCEL_CONFIG == MPU6500_ACCEL_FS_2G
  #define MPU6500_ACCEL_SENS  MPU6500_ACCEL_SENS_2G
#elif MPU6500_DEFAULT_ACCEL_CONFIG == MPU6500_ACCEL_FS_4G
  #define MPU6500_ACCEL_SENS  MPU6500_ACCEL_SENS_4G
#elif MPU6500_DEFAULT_ACCEL_CONFIG == MPU6500_ACCEL_FS_8G
  #define MPU6500_ACCEL_SENS  MPU6500_ACCEL_SENS_8G
#elif MPU6500_DEFAULT_ACCEL_CONFIG == MPU6500_ACCEL_FS_16G
  #define MPU6500_ACCEL_SENS  MPU6500_ACCEL_SENS_16G
#else
  #error "Invalid accelerometer configuration"
#endif

#define MPU6500_INT_Pin        MPU_INT_Pin
#define MPU6500_INT_GPIO_Port  MPU_INT_GPIO_Port

/* MPU6500 I2C Address */
#define MPU6500_ADDR		0x69 // AD0 = 0 -> 0x68 || AD0 = 1 -> 0x69

/**
 * @brief Initialize the MPU6500 accelerometer and gyroscope    
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
HAL_StatusTypeDef MPU6500_Init(void);

/**
 * @brief Enable data ready interrupts from the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Enables RAW_RDY_EN bit in INT_ENABLE register
 */
HAL_StatusTypeDef MPU6500_EnableDataReadyInterrupts(void);

/**
 * @brief Disable data ready interrupts from the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Disables RAW_RDY_EN bit in INT_ENABLE register
 */
HAL_StatusTypeDef MPU6500_DisableDataReadyInterrupts(void);

/**
 * @brief Read the WHO_AM_I register of the MPU6500
 * @param whoami Pointer to store the value read from WHO_AM_I register
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure    
 */
HAL_StatusTypeDef MPU6500_ReadWhoAmI(uint8_t *whoami);

/**
 * @brief Read accelerometer data from MPU6500
 * @param x Pointer to store X-axis acceleration in g
 * @param y Pointer to store Y-axis acceleration in g
 * @param z Pointer to store Z-axis acceleration in g
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Reads 6 bytes starting from ACCEL_XOUT_H register
 *       Converts raw data to physical units using configured sensitivity
 */
HAL_StatusTypeDef MPU6500_ReadAccel(float *x, float *y, float *z);

/**
 * @brief Read gyroscope data from MPU6500
 * @param x Pointer to store X-axis gyroscope data in degrees per second
 * @param y Pointer to store Y-axis gyroscope data in degrees per second
 * @param z Pointer to store Z-axis gyroscope data in degrees per second
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Reads 6 bytes starting from GYRO_XOUT_H register
 *       Converts raw data to physical units using configured sensitivity
 */
HAL_StatusTypeDef MPU6500_ReadGyro(float *x, float *y, float *z);

/**
 * @brief Read temperature data from MPU6500
 * @param temp Pointer to store temperature data
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Reads 2 bytes starting from TEMP_OUT_H register
 *       Data is in 16-bit format, high byte first
 */
HAL_StatusTypeDef MPU6500_ReadTemp(int16_t *temp);

/**
 * @brief Put the MPU6500 into sleep mode to save power
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Sets SLEEP bit (bit 6) in PWR_MGMT_1 register
 */
HAL_StatusTypeDef MPU6500_Sleep(void);

/**
 * @brief Wake up the MPU6500 from sleep mode
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Clears SLEEP bit (bit 6) in PWR_MGMT_1 register
 */
HAL_StatusTypeDef MPU6500_WakeUp(void);

#ifdef __cplusplus
}
#endif

#endif