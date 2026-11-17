/**
 ******************************************************************************
 * @file    bmp280.c
 * @brief   Bosch BMP280 I2C Sensor Driver Implementation with exact Bosch
 *          integer & float compensation formulas, hypsometric altitude, and
 *          STM32F7 HAL I2C hardware binding.
 ******************************************************************************
 */

#include "bmp280.h"
#include <math.h>

/* ==================================================================== */
/* Core Driver Functions                                                */
/* ==================================================================== */

bmp280_status_t bmp280_reset(bmp280_t *dev)
{
    if (!dev || !dev->write) {
        return BMP280_ERR_INVALID_PARAM;
    }

    uint8_t cmd = BMP280_RESET_VALUE;
    if (dev->write(dev->dev_addr, BMP280_REG_RESET, &cmd, 1, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    /* Bosch Datasheet Section 1 / Table 1: t_startup max 2 ms after reset */
    if (dev->delay_ms) {
        dev->delay_ms(10);
    }

    return BMP280_OK;
}

bmp280_status_t bmp280_is_measuring(bmp280_t *dev, bool *is_busy)
{
    if (!dev || !dev->read || !is_busy) {
        return BMP280_ERR_INVALID_PARAM;
    }

    uint8_t status = 0;
    if (dev->read(dev->dev_addr, BMP280_REG_STATUS, &status, 1, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    *is_busy = ((status & (BMP280_STATUS_MEASURING | BMP280_STATUS_IM_UPDATE)) != 0);
    return BMP280_OK;
}

bmp280_status_t bmp280_set_config(bmp280_t *dev, const bmp280_config_t *config)
{
    if (!dev || !dev->write || !config) {
        return BMP280_ERR_INVALID_PARAM;
    }

    if ((config->mode > BMP280_MODE_NORMAL) ||
        (config->osrs_t > BMP280_OSRS_16X) ||
        (config->osrs_p > BMP280_OSRS_16X) ||
        (config->filter > BMP280_FILTER_16) ||
        (config->standby > BMP280_STANDBY_4000_MS)) {
        return BMP280_ERR_INVALID_PARAM;
    }

    /* Bosch Section 4.3.4/4.3.5: enter sleep mode before writing config register */
    uint8_t ctrl_meas_sleep = 0x00;
    if (dev->write(dev->dev_addr, BMP280_REG_CTRL_MEAS, &ctrl_meas_sleep, 1, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    /* Config register: standby [7:5], filter [4:2], spi3w_en [0] = 0 (I2C mode) */
    uint8_t reg_config = (uint8_t)(((config->standby & 0x07U) << 5) |
                                   ((config->filter & 0x07U) << 2));
    if (dev->write(dev->dev_addr, BMP280_REG_CONFIG, &reg_config, 1, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    /* Ctrl_meas register: osrs_t [7:5], osrs_p [4:2], mode [1:0] */
    uint8_t reg_ctrl_meas = (uint8_t)(((config->osrs_t & 0x07U) << 5) |
                                      ((config->osrs_p & 0x07U) << 2) |
                                      (config->mode & 0x03U));
    if (dev->write(dev->dev_addr, BMP280_REG_CTRL_MEAS, &reg_ctrl_meas, 1, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    dev->config = *config;
    return BMP280_OK;
}

bmp280_status_t bmp280_init(bmp280_t *dev)
{
    if (!dev || !dev->read || !dev->write) {
        return BMP280_ERR_INVALID_PARAM;
    }

    /* 1. Read and verify Chip ID (0xD0 -> 0x58) */
    uint8_t chip_id = 0;
    if (dev->read(dev->dev_addr, BMP280_REG_CHIP_ID, &chip_id, 1, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    if (chip_id != BMP280_CHIP_ID && chip_id != 0x56 && chip_id != 0x57) {
        return BMP280_ERR_CHIP_ID;
    }

    /* 2. Soft Reset */
    bmp280_status_t status = bmp280_reset(dev);
    if (status != BMP280_OK) {
        return status;
    }

    /* 3. Wait until NVM copy (im_update) is complete */
    uint32_t timeout_cnt = 20;
    bool ready = false;
    while (timeout_cnt--) {
        bool busy = false;
        status = bmp280_is_measuring(dev, &busy);
        if (status != BMP280_OK) {
            return status;
        }
        if (!busy) {
            ready = true;
            break;
        }
        if (dev->delay_ms) {
            dev->delay_ms(2);
        }
    }
    if (!ready) {
        return BMP280_ERR_TIMEOUT;
    }

    /* 4. Burst read 24 trimming parameters (0x88..0x9F) */
    uint8_t calib_raw[24] = {0};
    if (dev->read(dev->dev_addr, BMP280_REG_CALIB_START, calib_raw, 24, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    dev->calib.dig_T1 = (uint16_t)(((uint16_t)calib_raw[1] << 8) | calib_raw[0]);
    dev->calib.dig_T2 = (int16_t)(((uint16_t)calib_raw[3] << 8) | calib_raw[2]);
    dev->calib.dig_T3 = (int16_t)(((uint16_t)calib_raw[5] << 8) | calib_raw[4]);

    dev->calib.dig_P1 = (uint16_t)(((uint16_t)calib_raw[7] << 8) | calib_raw[6]);
    dev->calib.dig_P2 = (int16_t)(((uint16_t)calib_raw[9] << 8) | calib_raw[8]);
    dev->calib.dig_P3 = (int16_t)(((uint16_t)calib_raw[11] << 8) | calib_raw[10]);
    dev->calib.dig_P4 = (int16_t)(((uint16_t)calib_raw[13] << 8) | calib_raw[12]);
    dev->calib.dig_P5 = (int16_t)(((uint16_t)calib_raw[15] << 8) | calib_raw[14]);
    dev->calib.dig_P6 = (int16_t)(((uint16_t)calib_raw[17] << 8) | calib_raw[16]);
    dev->calib.dig_P7 = (int16_t)(((uint16_t)calib_raw[19] << 8) | calib_raw[18]);
    dev->calib.dig_P8 = (int16_t)(((uint16_t)calib_raw[21] << 8) | calib_raw[20]);
    dev->calib.dig_P9 = (int16_t)(((uint16_t)calib_raw[23] << 8) | calib_raw[22]);

    dev->t_fine = 0;

    /* 5. Standard high-resolution Normal mode: osrs_t=2x, osrs_p=16x, filter=16, standby=62.5ms */
    bmp280_config_t default_cfg = {
        .mode = BMP280_MODE_NORMAL,
        .osrs_t = BMP280_OSRS_2X,
        .osrs_p = BMP280_OSRS_16X,
        .filter = BMP280_FILTER_16,
        .standby = BMP280_STANDBY_62_5_MS
    };

    return bmp280_set_config(dev, &default_cfg);
}

bmp280_status_t bmp280_read_raw(bmp280_t *dev, int32_t *raw_temp, int32_t *raw_press)
{
    if (!dev || !dev->read || !raw_temp || !raw_press) {
        return BMP280_ERR_INVALID_PARAM;
    }

    uint8_t buf[6] = {0};
    /* Bosch Section 3.9: single burst read 0xF7..0xFC prevents mixing sample bytes */
    if (dev->read(dev->dev_addr, BMP280_REG_PRESS_MSB, buf, 6, dev->intf_ptr) != 0) {
        return BMP280_ERR_COMM;
    }

    *raw_press = (int32_t)((((uint32_t)buf[0]) << 12) | (((uint32_t)buf[1]) << 4) | (((uint32_t)buf[2]) >> 4));
    *raw_temp  = (int32_t)((((uint32_t)buf[3]) << 12) | (((uint32_t)buf[4]) << 4) | (((uint32_t)buf[5]) >> 4));

    return BMP280_OK;
}

bmp280_status_t bmp280_read_data(bmp280_t *dev, float *temperature_c, float *pressure_pa, float *altitude_m)
{
    int32_t raw_temp = 0;
    int32_t raw_press = 0;

    bmp280_status_t status = bmp280_read_raw(dev, &raw_temp, &raw_press);
    if (status != BMP280_OK) {
        return status;
    }

    /* Temperature compensation MUST run first to update dev->t_fine */
    float temp = bmp280_compensate_T_float(dev, raw_temp);
    float press = bmp280_compensate_P_float(dev, raw_press);

    if (temperature_c) {
        *temperature_c = temp;
    }
    if (pressure_pa) {
        *pressure_pa = press;
    }
    if (altitude_m) {
        *altitude_m = bmp280_calculate_altitude_hypsometric(press / 100.0f, temp, BMP280_STANDARD_SEA_LEVEL_HPA);
    }

    return BMP280_OK;
}

/* ==================================================================== */
/* Exact Bosch Compensation Formulas                                    */
/* ==================================================================== */

int32_t bmp280_compensate_T_int32(bmp280_t *dev, int32_t adc_T)
{
    if (!dev) {
        return 0;
    }

    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * ((int32_t)dev->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) * ((int32_t)dev->calib.dig_T3)) >> 14;
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bmp280_compensate_P_int64(bmp280_t *dev, int32_t adc_P)
{
    if (!dev) {
        return 0;
    }

    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

    return (uint32_t)p;
}

uint32_t bmp280_compensate_P_int32(bmp280_t *dev, int32_t adc_P)
{
    if (!dev) {
        return 0;
    }

    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)dev->t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dev->calib.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dev->calib.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dev->calib.dig_P4) << 16);
    var1 = (((dev->calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((int32_t)dev->calib.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dev->calib.dig_P1)) >> 15);

    if (var1 == 0) {
        return 0;
    }

    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000U) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }

    var1 = (((int32_t)dev->calib.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dev->calib.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dev->calib.dig_P7) >> 4));

    return p;
}

float bmp280_compensate_T_float(bmp280_t *dev, int32_t adc_T)
{
    if (!dev) {
        return 0.0f;
    }

    float var1, var2, T;
    var1 = (((float)adc_T) / 16384.0f - ((float)dev->calib.dig_T1) / 1024.0f) * ((float)dev->calib.dig_T2);
    var2 = ((((float)adc_T) / 131072.0f - ((float)dev->calib.dig_T1) / 8192.0f) *
            (((float)adc_T) / 131072.0f - ((float)dev->calib.dig_T1) / 8192.0f)) * ((float)dev->calib.dig_T3);
    dev->t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0f;
    return T;
}

float bmp280_compensate_P_float(bmp280_t *dev, int32_t adc_P)
{
    if (!dev) {
        return 0.0f;
    }

    float var1, var2, p;
    var1 = ((float)dev->t_fine / 2.0f) - 64000.0f;
    var2 = var1 * var1 * ((float)dev->calib.dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)dev->calib.dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)dev->calib.dig_P4) * 65536.0f);
    var1 = (((float)dev->calib.dig_P3) * var1 * var1 / 524288.0f + ((float)dev->calib.dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)dev->calib.dig_P1);

    if (var1 == 0.0f) {
        return 0.0f;
    }

    p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)dev->calib.dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)dev->calib.dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)dev->calib.dig_P7)) / 16.0f;

    return p;
}

float bmp280_calculate_altitude(float press_hpa, float sea_level_hpa)
{
    if (sea_level_hpa <= 0.0f) {
        sea_level_hpa = BMP280_STANDARD_SEA_LEVEL_HPA;
    }
    if (press_hpa <= 0.0f) {
        return 0.0f;
    }

    return 44330.0f * (1.0f - powf(press_hpa / sea_level_hpa, 0.1902949f));
}

float bmp280_calculate_altitude_hypsometric(float press_hpa, float temp_c, float sea_level_hpa)
{
    if (sea_level_hpa <= 0.0f) {
        sea_level_hpa = BMP280_STANDARD_SEA_LEVEL_HPA;
    }
    if (press_hpa <= 0.0f) {
        return 0.0f;
    }
    if (temp_c < -273.15f) {
        temp_c = 15.0f;
    }

    float t_kelvin = temp_c + 273.15f;
    return (t_kelvin / 0.0065f) * (1.0f - powf(press_hpa / sea_level_hpa, 0.1902949f));
}

#ifndef UNIT_TEST
/* ==================================================================== */
/* STM32F7 HAL I2C Hardware Adaptation Layer (Bosch Section 5.2)        */
/* ==================================================================== */

static int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;
    if (!hi2c || !data || len == 0U) {
        return -1;
    }
    /* Bosch Section 5.2.2: Start -> Addr+W -> Reg -> Re-Start -> Addr+R -> N bytes */
    if (HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_addr << 1), (uint16_t)reg_addr,
                         I2C_MEMADD_SIZE_8BIT, data, len, 100U) != HAL_OK) {
        return -1;
    }
    return 0;
}

static int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;
    if (!hi2c || !data || len == 0U) {
        return -1;
    }
    /* Bosch Section 5.2.1: I2C write does not auto-increment; write [reg, val] per byte */
    for (uint16_t i = 0U; i < len; i++) {
        uint8_t val = data[i];
        if (HAL_I2C_Mem_Write(hi2c, (uint16_t)(dev_addr << 1), (uint16_t)((uint8_t)(reg_addr + i)),
                              I2C_MEMADD_SIZE_8BIT, &val, 1U, 100U) != HAL_OK) {
            return -1;
        }
    }
    return 0;
}

static void stm32_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

bmp280_status_t bmp280_init_default(bmp280_t *dev, void *hi2c, uint8_t dev_addr)
{
    if (!dev || !hi2c ||
        (dev_addr != BMP280_I2C_ADDR_PRIM && dev_addr != BMP280_I2C_ADDR_SEC)) {
        return BMP280_ERR_INVALID_PARAM;
    }

    dev->dev_addr = dev_addr;
    dev->read = stm32_i2c_read;
    dev->write = stm32_i2c_write;
    dev->delay_ms = stm32_delay_ms;
    dev->intf_ptr = hi2c;

    return bmp280_init(dev);
}
#endif
