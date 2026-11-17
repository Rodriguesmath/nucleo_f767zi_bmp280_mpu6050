/**
 ******************************************************************************
 * @file    bmp280.h
 * @brief   Bosch BMP280 Barometric Pressure & Temperature I2C Sensor Driver
 *          for NUCLEO-F767ZI (STM32F767ZIT6) and host unit testing.
 ******************************************************************************
 */

#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef UNIT_TEST
#include "stm32f7xx_hal.h"
#endif

/* ==================================================================== */
/* Device Constants & I2C Addresses (Bosch Datasheet Section 5.2)       */
/* ==================================================================== */
#define BMP280_I2C_ADDR_PRIM       0x76U  /*!< SDO pin pulled to GND */
#define BMP280_I2C_ADDR_SEC        0x77U  /*!< SDO pin pulled to VDDIO */

#define BMP280_CHIP_ID             0x58U  /*!< Production BMP280 chip ID */
#define BMP280_RESET_VALUE         0xB6U  /*!< Power-on-reset trigger command */

#define BMP280_STANDARD_SEA_LEVEL_PA  101325.0f /*!< Standard sea level pressure in Pa */
#define BMP280_STANDARD_SEA_LEVEL_HPA 1013.25f  /*!< Standard sea level pressure in hPa */

/* ==================================================================== */
/* Register Addresses (Bosch Datasheet Section 4.2)                     */
/* ==================================================================== */
#define BMP280_REG_CALIB_START     0x88U  /*!< Trimming parameters start (0x88..0x9F, 24 bytes) */
#define BMP280_REG_CHIP_ID         0xD0U  /*!< Chip ID register */
#define BMP280_REG_RESET           0xE0U  /*!< Soft reset register */
#define BMP280_REG_STATUS          0xF3U  /*!< Status register: measuring [3], im_update [0] */
#define BMP280_REG_CTRL_MEAS       0xF4U  /*!< osrs_t [7:5], osrs_p [4:2], mode [1:0] */
#define BMP280_REG_CONFIG          0xF5U  /*!< t_sb [7:5], filter [4:2], spi3w_en [0] (0 for I2C) */
#define BMP280_REG_PRESS_MSB       0xF7U  /*!< Pressure MSB [19:12] */
#define BMP280_REG_PRESS_LSB       0xF8U  /*!< Pressure LSB [11:4] */
#define BMP280_REG_PRESS_XLSB      0xF9U  /*!< Pressure XLSB [3:0] */
#define BMP280_REG_TEMP_MSB        0xFAU  /*!< Temperature MSB [19:12] */
#define BMP280_REG_TEMP_LSB        0xFBU  /*!< Temperature LSB [11:4] */
#define BMP280_REG_TEMP_XLSB       0xFCU  /*!< Temperature XLSB [3:0] */

/* ==================================================================== */
/* Status Flags                                                         */
/* ==================================================================== */
#define BMP280_STATUS_MEASURING    (1U << 3)
#define BMP280_STATUS_IM_UPDATE    (1U << 0)

/* ==================================================================== */
/* Configuration Types                                                  */
/* ==================================================================== */

typedef enum {
    BMP280_MODE_SLEEP  = 0x00,
    BMP280_MODE_FORCED = 0x01,
    BMP280_MODE_NORMAL = 0x03
} bmp280_mode_t;

typedef enum {
    BMP280_OSRS_SKIPPED = 0x00, /*!< Output disabled */
    BMP280_OSRS_1X      = 0x01, /*!< Ultra low power (x1) */
    BMP280_OSRS_2X      = 0x02, /*!< Low power (x2) */
    BMP280_OSRS_4X      = 0x03, /*!< Standard resolution (x4) */
    BMP280_OSRS_8X      = 0x04, /*!< High resolution (x8) */
    BMP280_OSRS_16X     = 0x05  /*!< Ultra high resolution (x16) */
} bmp280_oversampling_t;

typedef enum {
    BMP280_FILTER_OFF = 0x00,
    BMP280_FILTER_2   = 0x01,
    BMP280_FILTER_4   = 0x02,
    BMP280_FILTER_8   = 0x03,
    BMP280_FILTER_16  = 0x04
} bmp280_filter_t;

typedef enum {
    BMP280_STANDBY_0_5_MS  = 0x00, /*!< 0.5 ms */
    BMP280_STANDBY_62_5_MS = 0x01, /*!< 62.5 ms */
    BMP280_STANDBY_125_MS  = 0x02, /*!< 125 ms */
    BMP280_STANDBY_250_MS  = 0x03, /*!< 250 ms */
    BMP280_STANDBY_500_MS  = 0x04, /*!< 500 ms */
    BMP280_STANDBY_1000_MS = 0x05, /*!< 1000 ms */
    BMP280_STANDBY_2000_MS = 0x06, /*!< 2000 ms */
    BMP280_STANDBY_4000_MS = 0x07  /*!< 4000 ms */
} bmp280_standby_t;

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

typedef struct {
    bmp280_mode_t mode;
    bmp280_oversampling_t osrs_t;
    bmp280_oversampling_t osrs_p;
    bmp280_filter_t filter;
    bmp280_standby_t standby;
} bmp280_config_t;

typedef enum {
    BMP280_OK                =  0,
    BMP280_ERR_COMM          = -1,
    BMP280_ERR_CHIP_ID       = -2,
    BMP280_ERR_INVALID_PARAM = -3,
    BMP280_ERR_TIMEOUT       = -4
} bmp280_status_t;

/* Communication function pointers */
typedef int8_t (*bmp280_read_fp)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr);
typedef int8_t (*bmp280_write_fp)(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr);
typedef void (*bmp280_delay_ms_fp)(uint32_t ms);

/**
 * @brief BMP280 Device Handle
 */
typedef struct {
    uint8_t dev_addr;             /*!< 7-bit I2C device address (0x76 or 0x77) */
    bmp280_calib_t calib;         /*!< Factory calibration trimming parameters */
    int32_t t_fine;               /*!< Fine temperature compensation value */
    bmp280_config_t config;       /*!< Device configuration */
    bmp280_read_fp read;          /*!< Hardware read callback (0 on success) */
    bmp280_write_fp write;        /*!< Hardware write callback (0 on success) */
    bmp280_delay_ms_fp delay_ms;  /*!< Hardware delay callback */
    void *intf_ptr;               /*!< Interface pointer (e.g. &hi2c1) */
} bmp280_t;

/* ==================================================================== */
/* Function Prototypes                                                  */
/* ==================================================================== */

bmp280_status_t bmp280_init(bmp280_t *dev);
bmp280_status_t bmp280_reset(bmp280_t *dev);
bmp280_status_t bmp280_set_config(bmp280_t *dev, const bmp280_config_t *config);
bmp280_status_t bmp280_is_measuring(bmp280_t *dev, bool *is_busy);
bmp280_status_t bmp280_read_raw(bmp280_t *dev, int32_t *raw_temp, int32_t *raw_press);
bmp280_status_t bmp280_read_data(bmp280_t *dev, float *temperature_c, float *pressure_pa, float *altitude_m);

/* ==================================================================== */
/* Exact Bosch Compensation Formulas (Datasheet Section 8.1 / 8.2)      */
/* ==================================================================== */

int32_t bmp280_compensate_T_int32(bmp280_t *dev, int32_t adc_T);
uint32_t bmp280_compensate_P_int64(bmp280_t *dev, int32_t adc_P);
uint32_t bmp280_compensate_P_int32(bmp280_t *dev, int32_t adc_P);
float bmp280_compensate_T_float(bmp280_t *dev, int32_t adc_T);
float bmp280_compensate_P_float(bmp280_t *dev, int32_t adc_P);
float bmp280_calculate_altitude(float press_hpa, float sea_level_hpa);
float bmp280_calculate_altitude_hypsometric(float press_hpa, float temp_c, float sea_level_hpa);

#ifndef UNIT_TEST
/**
 * @brief Convenience initializer binding STM32 HAL I2C and Bosch default normal mode config.
 * @param dev Pointer to driver handle.
 * @param hi2c Pointer to I2C_HandleTypeDef (e.g. &hi2c1).
 * @param dev_addr 7-bit I2C address (BMP280_I2C_ADDR_PRIM=0x76 or BMP280_I2C_ADDR_SEC=0x77).
 * @return BMP280_OK on success, or error code.
 */
bmp280_status_t bmp280_init_default(bmp280_t *dev, void *hi2c, uint8_t dev_addr);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
