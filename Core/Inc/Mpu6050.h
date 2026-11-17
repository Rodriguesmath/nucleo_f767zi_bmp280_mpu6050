/******************************************************************************/
/**
 * @file    Mpu6050.h
 * @brief   Driver para o Sensor IMU de 6 Graus de Liberdade MPU-6050 (I2C).
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/
#ifndef _MPU6050_H_
#define _MPU6050_H_

/*******************************************************************************
 * INCLUDES NECESSARIOS
 ******************************************************************************/
#include "AssertTypes.h"

#ifndef UNIT_TEST
#include "stm32f7xx_hal.h"
#endif

/*******************************************************************************
 * CONFIGURACOES
 ******************************************************************************/
#define dMPU6050_I2C_ADDR_7BIT            0x68U
#define dMPU6050_I2C_ADDR_ALT_7BIT        0x69U
#define dMPU6050_I2C_ADDR                 (dMPU6050_I2C_ADDR_7BIT << 1)

/*******************************************************************************
 * DEFINES PUBLICOS
 ******************************************************************************/
#define dMPU6050_WHO_AM_I_VALUE           0x68U
#define dMPU6050_REG_SMPLRT_DIV           0x19U
#define dMPU6050_REG_CONFIG               0x1AU
#define dMPU6050_REG_GYRO_CONFIG          0x1BU
#define dMPU6050_REG_ACCEL_CONFIG         0x1CU
#define dMPU6050_REG_ACCEL_XOUT_H         0x3BU
#define dMPU6050_REG_TEMP_OUT_H           0x41U
#define dMPU6050_REG_GYRO_XOUT_H          0x43U
#define dMPU6050_REG_PWR_MGMT_1           0x6BU
#define dMPU6050_REG_WHO_AM_I             0x75U

/*******************************************************************************
 * TIPOS DE DADOS PUBLICOS
 ******************************************************************************/
typedef int8_t (*mpu6050ReadFp_t)(u8 devAddr7Bit, u8 reg, u8 *pBuffer, u16 len, void *pCtx);
typedef int8_t (*mpu6050WriteFp_t)(u8 devAddr7Bit, u8 reg, const u8 *pData, u16 len, void *pCtx);

typedef struct
{
    float accelXG;
    float accelYG;
    float accelZG;
    float gyroXDps;
    float gyroYDps;
    float gyroZDps;
    float tempC;

    s16   rawAccelX;
    s16   rawAccelY;
    s16   rawAccelZ;
    s16   rawGyroX;
    s16   rawGyroY;
    s16   rawGyroZ;
    s16   rawTemp;
} mpu6050Data_t;

/*******************************************************************************
 * PROTOTIPOS PUBLICOS
 ******************************************************************************/
#ifndef UNIT_TEST
bool Mpu6050_Init(void *pI2cHandle);
#endif
bool Mpu6050_InitWithBus(mpu6050ReadFp_t readFp, mpu6050WriteFp_t writeFp, void *pCtx, u8 devAddr7Bit);
bool Mpu6050_ReadSensor(mpu6050Data_t *pData);
bool Mpu6050_Sleep(void);
bool Mpu6050_Wakeup(void);

#endif /* _MPU6050_H_ */
/** @} DOXYGEN GROUP TAG END OF FILE */
