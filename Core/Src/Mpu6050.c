/******************************************************************************/
/**
 * @file    Mpu6050.c
 * @brief   Implementacao do driver do sensor MPU-6050 compartilhando I2C HAL/callbacks.
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "Mpu6050.h"
#include <stddef.h>

/*******************************************************************************
 * DEFINES LOCAIS (fixos, apenas auxiliar para calculos)
 ******************************************************************************/
#define dACCEL_SCALE_FACTOR_2G            16384.0f
#define dGYRO_SCALE_FACTOR_250            131.0f
#define dTEMP_DIVISOR                     340.0f
#define dTEMP_OFFSET                      36.53f
#define dBURST_READ_LENGTH                14U
#define dI2C_TIMEOUT_MS                   100U

/*******************************************************************************
 * ESTRUTURAS DE DADOS LOCAIS
 ******************************************************************************/
static struct
{
    bool             isConnected;
    u8               devAddr7Bit;
    mpu6050ReadFp_t  read;
    mpu6050WriteFp_t write;
    void            *pCtx;
} mpu6050;

/*******************************************************************************
 * PROTOTIPOS LOCAIS
 ******************************************************************************/
static bool Mpu6050_WriteRegister(u8 reg, u8 val);
static bool Mpu6050_ReadRegisters(u8 reg, u8 *pBuffer, u16 len);

#ifndef UNIT_TEST
static int8_t Mpu6050_HalRead(u8 devAddr7Bit, u8 reg, u8 *pBuffer, u16 len, void *pCtx);
static int8_t Mpu6050_HalWrite(u8 devAddr7Bit, u8 reg, const u8 *pData, u16 len, void *pCtx);
#endif

/*******************************************************************************
 * FUNCOES PUBLICAS
 ******************************************************************************/

bool Mpu6050_InitWithBus(mpu6050ReadFp_t readFp, mpu6050WriteFp_t writeFp, void *pCtx, u8 devAddr7Bit)
{
    u8 whoAmI = 0U;

    if ((readFp == NULL) || (writeFp == NULL) ||
        ((devAddr7Bit != dMPU6050_I2C_ADDR_7BIT) && (devAddr7Bit != dMPU6050_I2C_ADDR_ALT_7BIT)))
    {
        mpu6050.isConnected = false;
        return false;
    }

    mpu6050.read = readFp;
    mpu6050.write = writeFp;
    mpu6050.pCtx = pCtx;
    mpu6050.devAddr7Bit = devAddr7Bit;
    mpu6050.isConnected = false;

    /* 1. Verifica identidade do sensor (esperado: 0x68) */
    if ((Mpu6050_ReadRegisters(dMPU6050_REG_WHO_AM_I, &whoAmI, 1U) == false) ||
        (whoAmI != dMPU6050_WHO_AM_I_VALUE))
    {
        return false;
    }

    /* 2. Acorda dispositivo (PWR_MGMT_1: 0x01 = Clock PLL Giroscopio X) */
    if (Mpu6050_WriteRegister(dMPU6050_REG_PWR_MGMT_1, 0x01U) == false)
    {
        return false;
    }

    /* 3. Taxa de amostragem interna: 100 Hz (SMPLRT_DIV = 9) */
    if (Mpu6050_WriteRegister(dMPU6050_REG_SMPLRT_DIV, 0x09U) == false)
    {
        return false;
    }

    /* 4. Filtro Passa-Baixas Digital (DLPF): 44 Hz */
    if (Mpu6050_WriteRegister(dMPU6050_REG_CONFIG, 0x03U) == false)
    {
        return false;
    }

    /* 5. Giroscopio +/- 250 graus/s */
    if (Mpu6050_WriteRegister(dMPU6050_REG_GYRO_CONFIG, 0x00U) == false)
    {
        return false;
    }

    /* 6. Acelerometro +/- 2g */
    if (Mpu6050_WriteRegister(dMPU6050_REG_ACCEL_CONFIG, 0x00U) == false)
    {
        return false;
    }

    mpu6050.isConnected = true;
    return true;
}

#ifndef UNIT_TEST
bool Mpu6050_Init(void *pI2cHandle)
{
    if (pI2cHandle == NULL)
    {
        mpu6050.isConnected = false;
        return false;
    }

    return Mpu6050_InitWithBus(Mpu6050_HalRead, Mpu6050_HalWrite, pI2cHandle, dMPU6050_I2C_ADDR_7BIT);
}
#endif

bool Mpu6050_ReadSensor(mpu6050Data_t *pData)
{
    u8 rawBuffer[dBURST_READ_LENGTH];

    if ((pData == NULL) || (mpu6050.isConnected == false))
    {
        return false;
    }

    /* Leitura em burst de todos os 14 registradores continuos (0x3B..0x48) */
    if (Mpu6050_ReadRegisters(dMPU6050_REG_ACCEL_XOUT_H, rawBuffer, dBURST_READ_LENGTH) == false)
    {
        return false;
    }

    /* Conversao de Big-Endian para s16 nativo */
    pData->rawAccelX = (s16)(((u16)rawBuffer[0] << 8) | rawBuffer[1]);
    pData->rawAccelY = (s16)(((u16)rawBuffer[2] << 8) | rawBuffer[3]);
    pData->rawAccelZ = (s16)(((u16)rawBuffer[4] << 8) | rawBuffer[5]);
    pData->rawTemp   = (s16)(((u16)rawBuffer[6] << 8) | rawBuffer[7]);
    pData->rawGyroX  = (s16)(((u16)rawBuffer[8] << 8) | rawBuffer[9]);
    pData->rawGyroY  = (s16)(((u16)rawBuffer[10] << 8) | rawBuffer[11]);
    pData->rawGyroZ  = (s16)(((u16)rawBuffer[12] << 8) | rawBuffer[13]);

    /* Conversao para unidades de engenharia (g, graus/s, graus Celsius) */
    pData->accelXG  = (float)pData->rawAccelX / dACCEL_SCALE_FACTOR_2G;
    pData->accelYG  = (float)pData->rawAccelY / dACCEL_SCALE_FACTOR_2G;
    pData->accelZG  = (float)pData->rawAccelZ / dACCEL_SCALE_FACTOR_2G;

    pData->gyroXDps = (float)pData->rawGyroX / dGYRO_SCALE_FACTOR_250;
    pData->gyroYDps = (float)pData->rawGyroY / dGYRO_SCALE_FACTOR_250;
    pData->gyroZDps = (float)pData->rawGyroZ / dGYRO_SCALE_FACTOR_250;

    pData->tempC    = ((float)pData->rawTemp / dTEMP_DIVISOR) + dTEMP_OFFSET;

    return true;
}

bool Mpu6050_Sleep(void)
{
    return Mpu6050_WriteRegister(dMPU6050_REG_PWR_MGMT_1, 0x40U);
}

bool Mpu6050_Wakeup(void)
{
    return Mpu6050_WriteRegister(dMPU6050_REG_PWR_MGMT_1, 0x01U);
}

/*******************************************************************************
 * FUNCOES LOCAIS
 ******************************************************************************/

static bool Mpu6050_WriteRegister(u8 reg, u8 val)
{
    if (mpu6050.write == NULL)
    {
        return false;
    }

    return (mpu6050.write(mpu6050.devAddr7Bit, reg, &val, 1U, mpu6050.pCtx) == 0);
}

static bool Mpu6050_ReadRegisters(u8 reg, u8 *pBuffer, u16 len)
{
    if ((mpu6050.read == NULL) || (pBuffer == NULL) || (len == 0U))
    {
        return false;
    }

    return (mpu6050.read(mpu6050.devAddr7Bit, reg, pBuffer, len, mpu6050.pCtx) == 0);
}

#ifndef UNIT_TEST
static int8_t Mpu6050_HalRead(u8 devAddr7Bit, u8 reg, u8 *pBuffer, u16 len, void *pCtx)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)pCtx;
    if ((hi2c == NULL) || (pBuffer == NULL) || (len == 0U))
    {
        return -1;
    }

    if (HAL_I2C_Mem_Read(hi2c, (u16)(devAddr7Bit << 1), (u16)reg,
                         I2C_MEMADD_SIZE_8BIT, pBuffer, len, dI2C_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

static int8_t Mpu6050_HalWrite(u8 devAddr7Bit, u8 reg, const u8 *pData, u16 len, void *pCtx)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)pCtx;
    if ((hi2c == NULL) || (pData == NULL) || (len == 0U))
    {
        return -1;
    }

    if (HAL_I2C_Mem_Write(hi2c, (u16)(devAddr7Bit << 1), (u16)reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)pData, len, dI2C_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    return 0;
}
#endif

/** @} DOXYGEN GROUP TAG END OF FILE */
