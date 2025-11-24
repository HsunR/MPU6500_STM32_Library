/**
 * @file mpu6500.c 
 * @brief MPU6500 accelerometer and gyroscope driver
 * @details This file contains the implementation of the
 *          MPU6500 accelerometer and gyroscope driver, supporting various
 *          operations.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.0
 * @date 2025-06-08
 */

#include "mpu6500.h"

/* MPU6500 Register Addresses */
#define SELF_TEST_X_GYRO	0x00
#define SELF_TEST_Y_GYRO	0x01
#define SELF_TEST_Z_GYRO	0x02
#define SELF_TEST_X_ACCEL	0x0D
#define SELF_TEST_Y_ACCEL	0x0E
#define SELF_TEST_Z_ACCEL	0x0F
#define XG_OFFSET_H			0x13
#define XG_OFFSET_L			0x14
#define YG_OFFSET_H			0x15
#define YG_OFFSET_L			0x16
#define ZG_OFFSET_H			0x17
#define ZG_OFFSET_L			0x18
#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define ACCEL_CONFIG_2		0x1D
#define LP_ACCEL_ODR		0x1E
#define WOM_THR				0x1F
#define FIFO_EN				0x23
#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO			0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI			0x35
#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG			0x37
#define INT_ENABLE			0x38
#define INT_STATUS			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define I2C_SLV0_DO			0x63
#define I2C_SLV1_DO			0x64
#define I2C_SLV2_DO			0x65
#define I2C_SLV3_DO			0x66
#define I2C_MST_DELAY_CTRL	0x67
#define SIGNAL_PATH_RESET	0x68
#define ACCEL_INTEL_CTRL	0x69
#define USER_CTRL			0x6A
#define PWR_MGMT_1			0x6B
#define PWR_MGMT_2			0x6C
#define FIFO_COUNT_H		0x72
#define FIFO_COUNT_L		0x73
#define FIFO_R_W			0x74
#define WHO_AM_I			0x75
#define XA_OFFSET_H			0x77
#define XA_OFFSET_L			0x78
#define YA_OFFSET_H			0x7A
#define YA_OFFSET_L			0x7B
#define ZA_OFFSET_H			0x7D
#define ZA_OFFSET_L			0x7E

int16_t accel_offset[3];
int16_t gyro_offset[3];

/**
 * @brief Write a single byte to an MPU6500 register
 * @param reg Register address to write to
 * @param data Data byte to write
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
static inline HAL_StatusTypeDef MPU6500_WriteRegister(uint8_t reg, uint8_t data){
    return HAL_I2C_Mem_Write(&hi2c1, (MPU6500_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Read a single byte from an MPU6500 register
 * @param reg Register address to read from
 * @param data Pointer to store read data
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
static inline HAL_StatusTypeDef MPU6500_ReadRegister(uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Read(&hi2c1, (MPU6500_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reset the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
static inline HAL_StatusTypeDef MPU6500_Reset(void){
    return MPU6500_WriteRegister(PWR_MGMT_1, 0x80); // DEVICE_RESET[7]
}

/**
 * @brief Configure the clock source of the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
static inline HAL_StatusTypeDef MPU6500_ConfigureClock(void){
    return MPU6500_WriteRegister(PWR_MGMT_1, 0x01); // SLEEP[6] | CLKSEL[2:0]
}

/**
 * @brief Configure the accelerometer
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Configuration sequence:
 *       1. Configure accelerometer full scale range
 *       2. Configure accelerometer low pass filter
 */
static inline HAL_StatusTypeDef MPU6500_ConfigureAccel(void){
    HAL_StatusTypeDef status;
    status = MPU6500_WriteRegister(ACCEL_CONFIG, MPU6500_DEFAULT_ACCEL_CONFIG); // ACCEL_FS_SEL[4:3] = 11 (±16g), bits [2:0] reserved (0)
    if(status != HAL_OK) return status;
    status = MPU6500_WriteRegister(ACCEL_CONFIG_2, 0x04); // ACCEL_DLPF_CFG[2:0] = 100 (20Hz, 1kHz)
    if(status != HAL_OK) return status;
    return HAL_OK;
}

/**
 * @brief Configure the gyroscope
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Configuration sequence:
 *       1. Configure gyroscope full scale range
 *       2. Configure gyroscope low pass filter
 */
static inline HAL_StatusTypeDef MPU6500_ConfigureGyro(void){
    HAL_StatusTypeDef status;
    status = MPU6500_WriteRegister(GYRO_CONFIG, MPU6500_DEFAULT_GYRO_CONFIG); // GYRO_FS_SEL[1:0] = 11 | FCHOICE_B[1:0] = 00 || Full scale range = ±2000dps
    if(status != HAL_OK) return status;
    status = MPU6500_WriteRegister(CONFIG, 0x04); // DLPF_CFG[2:0] = 100 || Gyroscope low pass filter Bandwidth = 20Hz | Data Rate = 1kHz
    if(status != HAL_OK) return status;
    return HAL_OK;  
} 

/**
 * @brief Disable the gyroscope of the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
static inline HAL_StatusTypeDef MPU6500_DisableGyro(void){
    return MPU6500_WriteRegister(PWR_MGMT_2, 0x07); // DISABLE_XG[2]|DISABLE_YG[1]|DISABLE_ZG[0]
}

/**
 * @brief Enable the temperature sensor of the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
static inline HAL_StatusTypeDef MPU6500_EnableTemperatureSensor(void){
    HAL_StatusTypeDef status;
    uint8_t regData;
    // Read current PWR_MGMT_1 register
    status = MPU6500_ReadRegister(PWR_MGMT_1, &regData);
    if(status != HAL_OK) return status;
    // Clear TEMP_DIS bit (bit 4)
    regData &= ~(1 << 4);
    // Write back to PWR_MGMT_1
    return MPU6500_WriteRegister(PWR_MGMT_1, regData);
}

/**
 * @brief Disable the temperature sensor of the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
static inline HAL_StatusTypeDef MPU6500_DisableTemperatureSensor(void){
    HAL_StatusTypeDef status;
    uint8_t regData;
    // Read current PWR_MGMT_1 register
    status = MPU6500_ReadRegister(PWR_MGMT_1, &regData);
    if(status != HAL_OK) return status;
    // Set TEMP_DIS bit (bit 4)
    regData |= (1 << 4);
    // Write back to PWR_MGMT_1
    return MPU6500_WriteRegister(PWR_MGMT_1, regData);
}   

/**
 * @brief Configure the interrupt pin
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */ 
static inline HAL_StatusTypeDef MPU6500_ConfigureInterrupts(void){
    HAL_StatusTypeDef status;
    status = MPU6500_WriteRegister(INT_PIN_CFG, 0xB0); // ACTL[7] | OPEN[6] | LATCH_INT_EN[5] | INT_ANYRD_2CLEAR[4]
    if(status != HAL_OK) return status;
    return HAL_OK;
}

/**
 * @brief Initialize the MPU6500 accelerometer
 * @return HAL_StatusTypeDef HAL_OK on success, error code on failure
 * @note Configuration sequence:
 *       1. Reset device
 *       2. Wake up and configure clock
 *       3. Configure accelerometer (±16g, 20Hz bandwidth)
 *       4. Configure gyroscope (±2000dps, 20Hz bandwidth)
 *       5. Enable temperature sensor
 *       6. Configure interrupt pin
 */
HAL_StatusTypeDef MPU6500_Init(void){
    HAL_StatusTypeDef status;
    // 1. Reset device
    status = MPU6500_Reset();
    if (status != HAL_OK) return status;
    HAL_Delay(100); // Wait for reset to complete
    // 2. Wake up device and select clock source
    status = MPU6500_ConfigureClock();
    if(status != HAL_OK) return status;
    // 3. Configure Accelerometer
    status = MPU6500_ConfigureAccel();
    if(status != HAL_OK) return status;
    // 4. Configure Gyroscope
    status = MPU6500_ConfigureGyro();
    if(status != HAL_OK) return status;
    // 5. Enable temperature sensor
    status = MPU6500_EnableTemperatureSensor();
    if(status != HAL_OK) return status; 
    // 6. Configure INT Pin (but don't enable interrupts yet)
    status = MPU6500_ConfigureInterrupts();
    if(status != HAL_OK) return status;
    return HAL_OK;
}

/**
 * @brief Enable data ready interrupts from the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Enables RAW_RDY_EN bit in INT_ENABLE register
 */
HAL_StatusTypeDef MPU6500_EnableDataReadyInterrupts(void){
    return MPU6500_WriteRegister(INT_ENABLE, 0x01); // RAW_RDY_EN[0]
}

/**
 * @brief Disable data ready interrupts from the MPU6500
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Disables RAW_RDY_EN bit in INT_ENABLE register
 */
HAL_StatusTypeDef MPU6500_DisableDataReadyInterrupts(void){
    return MPU6500_WriteRegister(INT_ENABLE, 0x00); // RAW_RDY_EN[0]
}

/**
 * @brief Read the WHO_AM_I register of the MPU6500
 * @param whoami Pointer to store the value read from WHO_AM_I register
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 */
HAL_StatusTypeDef MPU6500_ReadWhoAmI(uint8_t *whoami){  
    return MPU6500_ReadRegister(WHO_AM_I, whoami);
}

HAL_StatusTypeDef MPU6500_ReadRawAccel(int16_t *x, int16_t *y, int16_t *z){
    HAL_StatusTypeDef status;
    uint8_t buffer[6];  // 6 bytes for data
    // Read all 6 bytes starting from ACCEL_XOUT_H
    status = HAL_I2C_Mem_Read(&hi2c1, (MPU6500_ADDR << 1), ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);
    if(status != HAL_OK) return status;
    // Combine bytes into 16-bit values (high byte first, then low byte)
    *x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *z = (int16_t)((buffer[4] << 8) | buffer[5]);
    return HAL_OK;
}   

/**
 * @brief Read gyroscope data from MPU6500
 * @param x Pointer to store X-axis gyroscope data
 * @param y Pointer to store Y-axis gyroscope data
 * @param z Pointer to store Z-axis gyroscope data
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Reads 6 bytes starting from GYRO_XOUT_H register
 *       Data is in 16-bit format, high byte first
 */
HAL_StatusTypeDef MPU6500_ReadRawGyro(int16_t *x, int16_t *y, int16_t *z){
    HAL_StatusTypeDef status;
    uint8_t buffer[6];  // 6 bytes for data
    // Read all 6 bytes starting from GYRO_XOUT_H
    status = HAL_I2C_Mem_Read(&hi2c1, (MPU6500_ADDR << 1), GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);
    if(status != HAL_OK) return status;
    // Combine bytes into 16-bit values (high byte first, then low byte)
    *x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *z = (int16_t)((buffer[4] << 8) | buffer[5]);
    return HAL_OK;
}       

/**
 * @brief Read accelerometer data from MPU6500
 * @param x Pointer to store X-axis acceleration in g
 * @param y Pointer to store Y-axis acceleration in g
 * @param z Pointer to store Z-axis acceleration in g
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Reads 6 bytes starting from ACCEL_XOUT_H register
 *       Converts raw data to physical units using configured sensitivity
 */
HAL_StatusTypeDef MPU6500_ReadAccel(float *x, float *y, float *z){
    HAL_StatusTypeDef status;
    uint8_t buffer[6];  // 6 bytes for data
    int16_t raw_x, raw_y, raw_z;
    
    // Read all 6 bytes starting from ACCEL_XOUT_H
    status = HAL_I2C_Mem_Read(&hi2c1, (MPU6500_ADDR << 1), ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);
    if(status != HAL_OK) return status;
    
    // Combine bytes into 16-bit values (high byte first, then low byte)
    raw_x = (int16_t)((buffer[0] << 8) | buffer[1]) - accel_offset[0];
    raw_y = (int16_t)((buffer[2] << 8) | buffer[3]) - accel_offset[1];
    raw_z = (int16_t)((buffer[4] << 8) | buffer[5]) - accel_offset[2];
    
    // Convert to physical units (g)
    *x = (float)raw_x / MPU6500_ACCEL_SENS;
    *y = (float)raw_y / MPU6500_ACCEL_SENS;
    *z = (float)raw_z / MPU6500_ACCEL_SENS;
    
    return HAL_OK;
}

/**
 * @brief Read gyroscope data from MPU6500
 * @param x Pointer to store X-axis gyroscope data in degrees per second
 * @param y Pointer to store Y-axis gyroscope data in degrees per second
 * @param z Pointer to store Z-axis gyroscope data in degrees per second
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Reads 6 bytes starting from GYRO_XOUT_H register
 *       Converts raw data to physical units using configured sensitivity
 */
HAL_StatusTypeDef MPU6500_ReadGyro(float *x, float *y, float *z){
    HAL_StatusTypeDef status;
    uint8_t buffer[6];  // 6 bytes for data
    int16_t raw_x, raw_y, raw_z;
    
    // Read all 6 bytes starting from GYRO_XOUT_H
    status = HAL_I2C_Mem_Read(&hi2c1, (MPU6500_ADDR << 1), GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);
    if(status != HAL_OK) return status;
    
    // Combine bytes into 16-bit values (high byte first, then low byte)
    raw_x = (int16_t)((buffer[0] << 8) | buffer[1]) - gyro_offset[0];
    raw_y = (int16_t)((buffer[2] << 8) | buffer[3]) - gyro_offset[1];
    raw_z = (int16_t)((buffer[4] << 8) | buffer[5]) - gyro_offset[2];
    
    // Convert to physical units (degrees per second)
    *x = (float)raw_x / MPU6500_GYRO_SENS;
    *y = (float)raw_y / MPU6500_GYRO_SENS;
    *z = (float)raw_z / MPU6500_GYRO_SENS;
    
    return HAL_OK;
}

/**
 * @brief Read temperature data from MPU6500
 * @param[out] temp Pointer to store the raw temperature value (signed 16-bit)
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Reads 2 bytes from TEMP_OUT_H and TEMP_OUT_L.
 *       The value is in 16-bit signed format (big endian).
 *       Conversion to Celsius: Temp(°C) = temp / 333.87 + 21
 */
HAL_StatusTypeDef MPU6500_ReadTemp(int16_t *temp){
    HAL_StatusTypeDef status;
    uint8_t buffer[2];

    // Read 2 bytes starting from TEMP_OUT_H
    status = HAL_I2C_Mem_Read(&hi2c1, (MPU6500_ADDR << 1), TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Combine bytes into signed 16-bit integer
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);

    return HAL_OK;
}

/**
 * @brief Put the MPU6500 into sleep mode to save power
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Sets SLEEP bit (bit 6) in PWR_MGMT_1 register
 */
HAL_StatusTypeDef MPU6500_Sleep(void){
    HAL_StatusTypeDef status;
    uint8_t regData;
    // Read current PWR_MGMT_1 register
    status = MPU6500_ReadRegister(PWR_MGMT_1, &regData);
    if(status != HAL_OK) return status;
    // Set SLEEP bit (bit 6)
    regData |= (1 << 6);
    // Write back to PWR_MGMT_1
    return MPU6500_WriteRegister(PWR_MGMT_1, regData);
}

/**
 * @brief Wake up the MPU6500 from sleep mode
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note Clears SLEEP bit (bit 6) in PWR_MGMT_1 register
 */
HAL_StatusTypeDef MPU6500_WakeUp(void){
    HAL_StatusTypeDef status;
    uint8_t regData;
    // Read current PWR_MGMT_1 register
    status = MPU6500_ReadRegister(PWR_MGMT_1, &regData);
    if(status != HAL_OK) return status;
    // Clear SLEEP bit (bit 6)
    regData &= ~(1 << 6);
    // Write back to PWR_MGMT_1
    return MPU6500_WriteRegister(PWR_MGMT_1, regData);
}


HAL_StatusTypeDef MPU6500_InitOffsetCalibration(uint32_t samples) {
    HAL_StatusTypeDef status = HAL_OK;
    int32_t accel_sum[3] = {0};
    int32_t gyro_sum[3] = {0};
    uint32_t i;
    
    // 验证参数有效性
    if (samples == 0) {
        return HAL_ERROR;
    }
    
    // 确保传感器已初始化并处于活跃状态
    status = MPU6500_WakeUp();
    if (status != HAL_OK) {
        return status;
    }
    
    // 收集样本数据
    for (i = 0; i < samples; i++) {
        int16_t raw_accel[3];
        int16_t raw_gyro[3];
        
        // 读取原始加速度计数据
        status = MPU6500_ReadRawAccel(&raw_accel[0], &raw_accel[1], &raw_accel[2]);
        if (status != HAL_OK) {
            return status;
        }
        
        // 读取原始陀螺仪数据
        status = MPU6500_ReadRawGyro(&raw_gyro[0], &raw_gyro[1], &raw_gyro[2]);
        if (status != HAL_OK) {
            return status;
        }
        
        // 累加数据（注意：对于Z轴加速度，我们期望它接近1g，所以需要减去1g的原始值）
        accel_sum[0] += raw_accel[0];
        accel_sum[1] += raw_accel[1];
        accel_sum[2] += (raw_accel[2] - (int16_t)(1.0f * MPU6500_ACCEL_SENS));
        
        // 累加陀螺仪数据
        gyro_sum[0] += raw_gyro[0];
        gyro_sum[1] += raw_gyro[1];
        gyro_sum[2] += raw_gyro[2];
        
        // 短暂延迟以确保采样均匀
        HAL_Delay(5);
    }
    
    // 计算平均偏移值
    accel_offset[0] = (int16_t)(accel_sum[0] / samples);
    accel_offset[1] = (int16_t)(accel_sum[1] / samples);
    accel_offset[2] = (int16_t)(accel_sum[2] / samples);
    
    gyro_offset[0] = (int16_t)(gyro_sum[0] / samples);
    gyro_offset[1] = (int16_t)(gyro_sum[1] / samples);
    gyro_offset[2] = (int16_t)(gyro_sum[2] / samples);
    
    return HAL_OK;
}

/**
 * @brief 打印MPU6500的偏移校准值
 * @return HAL_StatusTypeDef HAL_OK on success, error on failure
 * @note 此函数将打印加速度计和陀螺仪的偏移校准值到串口
 */
HAL_StatusTypeDef MPU6500_PrintOffsets(void) {
    // 打印加速度计偏移值
    printf("Accelerometer Offsets:\n");
    printf("X: %d\n", accel_offset[0]);
    printf("Y: %d\n", accel_offset[1]);
    printf("Z: %d\n", accel_offset[2]);
    
    // 打印陀螺仪偏移值
    printf("Gyroscope Offsets:\n");
    printf("X: %d\n", gyro_offset[0]);
    printf("Y: %d\n", gyro_offset[1]);
    printf("Z: %d\n", gyro_offset[2]);
    
    return HAL_OK;
}