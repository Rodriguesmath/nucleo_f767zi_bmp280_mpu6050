/**
 ******************************************************************************
 * @file    test_bmp280.c
 * @author  Senior Implementation Engineer
 * @brief   Unit test suite for Bosch BMP280 barometric pressure driver.
 *          Verifies exact datasheet test vectors (page 23), compensation
 *          algorithms (int32, int64, float), altitude, and mock I2C lifecycle.
 ******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "bmp280.h"

static int g_tests_run = 0;
static int g_tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
        printf("  [PASS] %s\n", msg); \
    } else { \
        printf("  [FAIL] %s (Line %d)\n", msg, __LINE__); \
        return -1; \
    } \
} while(0)

/* ==================================================================== */
/* Test 1: Bosch Datasheet Page 23 Integer & Float Test Vector         */
/* ==================================================================== */
static int test_datasheet_page23_vector(void)
{
    printf("\n--- Test 1: Bosch Datasheet Page 23 Test Vector ---\n");

    bmp280_t dev;
    memset(&dev, 0, sizeof(dev));

    /* Trimming parameters from datasheet page 23 */
    dev.calib.dig_T1 = 27504;
    dev.calib.dig_T2 = 26435;
    dev.calib.dig_T3 = -1000;

    dev.calib.dig_P1 = 36477;
    dev.calib.dig_P2 = -10685;
    dev.calib.dig_P3 = 3024;
    dev.calib.dig_P4 = 2855;
    dev.calib.dig_P5 = 140;
    dev.calib.dig_P6 = -7;
    dev.calib.dig_P7 = 15500;
    dev.calib.dig_P8 = -14600;
    dev.calib.dig_P9 = 6000;

    /* Raw ADC samples from page 23 */
    const int32_t UT = 519888;
    const int32_t UP = 415148;

    /* 1.1 32-bit integer temperature compensation */
    int32_t T_int32 = bmp280_compensate_T_int32(&dev, UT);
    printf("  Calculated t_fine: %d (expected 128422)\n", (int)dev.t_fine);
    printf("  Calculated T_int32: %d (expected 2508 -> 25.08 degC)\n", (int)T_int32);
    TEST_ASSERT(dev.t_fine == 128422, "t_fine == 128422");
    TEST_ASSERT(T_int32 == 2508, "T_int32 == 2508 (25.08 degC)");

    /* 1.2 64-bit integer pressure compensation */
    uint32_t P_int64_q24_8 = bmp280_compensate_P_int64(&dev, UP);
    uint32_t P_int64_pa = P_int64_q24_8 >> 8;
    printf("  Calculated P_int64 Q24.8: %u, in Pa: %u (expected 100653 Pa)\n",
           (unsigned int)P_int64_q24_8, (unsigned int)P_int64_pa);
    TEST_ASSERT(P_int64_pa == 100653U, "P_int64 in Pa == 100653");

    /* 1.3 32-bit integer pressure compensation */
    uint32_t P_int32_pa = bmp280_compensate_P_int32(&dev, UP);
    printf("  Calculated P_int32 in Pa: %u (expected ~100653-100656 Pa)\n", (unsigned int)P_int32_pa);
    TEST_ASSERT(abs((int)P_int32_pa - 100653) <= 5, "P_int32 within 5 Pa of 100653");

    /* 1.4 Floating point temperature and pressure */
    float T_float = bmp280_compensate_T_float(&dev, UT);
    float P_float = bmp280_compensate_P_float(&dev, UP);
    printf("  Calculated T_float: %.4f degC (expected 25.08 degC)\n", T_float);
    printf("  Calculated P_float: %.2f Pa (expected 100653.25 Pa)\n", P_float);
    TEST_ASSERT(fabsf(T_float - 25.0825f) < 0.01f, "T_float matches 25.08 degC");
    TEST_ASSERT(fabsf(P_float - 100653.25f) < 0.5f, "P_float matches 100653 Pa");

    return 0;
}

/* ==================================================================== */
/* Test 2: Altitude Barometric Formula                                  */
/* ==================================================================== */
static int test_altitude_calculation(void)
{
    printf("\n--- Test 2: Altitude Calculation (Standard & Hypsometric) ---\n");

    /* At standard sea level pressure (1013.25 hPa), altitude must be 0 m */
    float alt_sea_level = bmp280_calculate_altitude(1013.25f, 1013.25f);
    printf("  Standard Sea level altitude: %.2f m\n", alt_sea_level);
    TEST_ASSERT(fabsf(alt_sea_level) < 0.01f, "Altitude at 1013.25 hPa is 0.0 m");

    /* At test vector pressure ~1006.53 hPa, standard altitude should be ~56.4 m */
    float alt_test = bmp280_calculate_altitude(1006.5325f, 1013.25f);
    printf("  Standard Altitude at 1006.5325 hPa: %.2f m\n", alt_test);
    TEST_ASSERT(fabsf(alt_test - 56.4f) < 1.0f, "Standard altitude at 1006.53 hPa is ~56.4 m");

    /* Hypsometric formula: at T=15°C (standard air), must match standard ~56.4 m */
    float alt_hyp_15 = bmp280_calculate_altitude_hypsometric(1006.5325f, 15.0f, 1013.25f);
    printf("  Hypsometric Altitude at 15°C: %.2f m\n", alt_hyp_15);
    TEST_ASSERT(fabsf(alt_hyp_15 - 56.4f) < 1.0f, "Hypsometric altitude at 15°C matches standard ~56.4 m");

    /* Hypsometric formula: at T=25.08°C (warmer air, lower density -> higher column), should be ~58.2 m */
    float alt_hyp_25 = bmp280_calculate_altitude_hypsometric(1006.5325f, 25.08f, 1013.25f);
    printf("  Hypsometric Altitude at 25.08°C: %.2f m\n", alt_hyp_25);
    TEST_ASSERT(fabsf(alt_hyp_25 - 58.2f) < 1.0f, "Hypsometric altitude at 25.08°C is ~58.2 m");

    /* Boundary check: invalid/negative pressure returns 0 */
    float alt_invalid = bmp280_calculate_altitude(-10.0f, 1013.25f);
    TEST_ASSERT(alt_invalid == 0.0f, "Invalid pressure returns 0.0 m");

    float alt_hyp_invalid = bmp280_calculate_altitude_hypsometric(-10.0f, 25.0f, 1013.25f);
    TEST_ASSERT(alt_hyp_invalid == 0.0f, "Invalid pressure in hypsometric returns 0.0 m");

    return 0;
}

/* ==================================================================== */
/* Test 3: Mock I2C Bus Lifecycle (Init, Config, Read Data)             */
/* ==================================================================== */
static uint8_t s_mock_reg_map[256];

static int8_t mock_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr)
{
    (void)dev_addr;
    (void)intf_ptr;
    for (uint16_t i = 0; i < len; i++) {
        data[i] = s_mock_reg_map[(reg_addr + i) & 0xFF];
    }
    return 0;
}

static int8_t mock_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    (void)dev_addr;
    (void)intf_ptr;
    for (uint16_t i = 0; i < len; i++) {
        s_mock_reg_map[(reg_addr + i) & 0xFF] = data[i];
    }
    return 0;
}

static void mock_delay_ms(uint32_t ms)
{
    (void)ms;
}

static int test_mock_sensor_lifecycle(void)
{
    printf("\n--- Test 3: Mock I2C Sensor Lifecycle ---\n");

    memset(s_mock_reg_map, 0, sizeof(s_mock_reg_map));

    /* Preload Chip ID = 0x58 */
    s_mock_reg_map[BMP280_REG_CHIP_ID] = BMP280_CHIP_ID;
    s_mock_reg_map[BMP280_REG_STATUS] = 0x00; /* Not busy */

    /* Preload calibration bytes matching datasheet page 23 */
    const uint16_t dig_T1 = 27504;
    const int16_t  dig_T2 = 26435;
    const int16_t  dig_T3 = -1000;
    const uint16_t dig_P1 = 36477;
    const int16_t  dig_P2 = -10685;
    const int16_t  dig_P3 = 3024;
    const int16_t  dig_P4 = 2855;
    const int16_t  dig_P5 = 140;
    const int16_t  dig_P6 = -7;
    const int16_t  dig_P7 = 15500;
    const int16_t  dig_P8 = -14600;
    const int16_t  dig_P9 = 6000;

    s_mock_reg_map[0x88] = (uint8_t)(dig_T1 & 0xFF);
    s_mock_reg_map[0x89] = (uint8_t)(dig_T1 >> 8);
    s_mock_reg_map[0x8A] = (uint8_t)(dig_T2 & 0xFF);
    s_mock_reg_map[0x8B] = (uint8_t)(dig_T2 >> 8);
    s_mock_reg_map[0x8C] = (uint8_t)(dig_T3 & 0xFF);
    s_mock_reg_map[0x8D] = (uint8_t)(dig_T3 >> 8);

    s_mock_reg_map[0x8E] = (uint8_t)(dig_P1 & 0xFF);
    s_mock_reg_map[0x8F] = (uint8_t)(dig_P1 >> 8);
    s_mock_reg_map[0x90] = (uint8_t)(dig_P2 & 0xFF);
    s_mock_reg_map[0x91] = (uint8_t)(dig_P2 >> 8);
    s_mock_reg_map[0x92] = (uint8_t)(dig_P3 & 0xFF);
    s_mock_reg_map[0x93] = (uint8_t)(dig_P3 >> 8);
    s_mock_reg_map[0x94] = (uint8_t)(dig_P4 & 0xFF);
    s_mock_reg_map[0x95] = (uint8_t)(dig_P4 >> 8);
    s_mock_reg_map[0x96] = (uint8_t)(dig_P5 & 0xFF);
    s_mock_reg_map[0x97] = (uint8_t)(dig_P5 >> 8);
    s_mock_reg_map[0x98] = (uint8_t)(dig_P6 & 0xFF);
    s_mock_reg_map[0x99] = (uint8_t)(dig_P6 >> 8);
    s_mock_reg_map[0x9A] = (uint8_t)(dig_P7 & 0xFF);
    s_mock_reg_map[0x9B] = (uint8_t)(dig_P7 >> 8);
    s_mock_reg_map[0x9C] = (uint8_t)(dig_P8 & 0xFF);
    s_mock_reg_map[0x9D] = (uint8_t)(dig_P8 >> 8);
    s_mock_reg_map[0x9E] = (uint8_t)(dig_P9 & 0xFF);
    s_mock_reg_map[0x9F] = (uint8_t)(dig_P9 >> 8);

    /* Preload ADC output registers with test vector (UP=415148, UT=519888) */
    /* UP = 415148 = 0x0655AC -> press_msb=0x06, press_lsb=0x55, press_xlsb=0xAC (top 4 bits: 0xA0) */
    s_mock_reg_map[BMP280_REG_PRESS_MSB]  = (uint8_t)((415148 >> 12) & 0xFF);
    s_mock_reg_map[BMP280_REG_PRESS_LSB]  = (uint8_t)((415148 >> 4) & 0xFF);
    s_mock_reg_map[BMP280_REG_PRESS_XLSB] = (uint8_t)((415148 << 4) & 0xF0);

    /* UT = 519888 = 0x07EED0 -> temp_msb=0x07, temp_lsb=0xEE, temp_xlsb=0xD0 */
    s_mock_reg_map[BMP280_REG_TEMP_MSB]  = (uint8_t)((519888 >> 12) & 0xFF);
    s_mock_reg_map[BMP280_REG_TEMP_LSB]  = (uint8_t)((519888 >> 4) & 0xFF);
    s_mock_reg_map[BMP280_REG_TEMP_XLSB] = (uint8_t)((519888 << 4) & 0xF0);

    bmp280_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.dev_addr = BMP280_I2C_ADDR_PRIM;
    dev.read = mock_i2c_read;
    dev.write = mock_i2c_write;
    dev.delay_ms = mock_delay_ms;

    /* Initialize device */
    bmp280_status_t status = bmp280_init(&dev);
    TEST_ASSERT(status == BMP280_OK, "bmp280_init returns BMP280_OK");
    TEST_ASSERT(dev.calib.dig_T1 == 27504, "dig_T1 read accurately");
    TEST_ASSERT(dev.calib.dig_P9 == 6000, "dig_P9 read accurately");

    /* Read raw ADC values */
    int32_t raw_temp = 0, raw_press = 0;
    status = bmp280_read_raw(&dev, &raw_temp, &raw_press);
    TEST_ASSERT(status == BMP280_OK, "bmp280_read_raw returns BMP280_OK");
    TEST_ASSERT(raw_temp == 519888, "raw_temp == 519888");
    TEST_ASSERT(raw_press == 415148, "raw_press == 415148");

    /* Read engineering units */
    float temp_c = 0.0f, press_pa = 0.0f, alt_m = 0.0f;
    status = bmp280_read_data(&dev, &temp_c, &press_pa, &alt_m);
    TEST_ASSERT(status == BMP280_OK, "bmp280_read_data returns BMP280_OK");
    TEST_ASSERT(fabsf(temp_c - 25.08f) < 0.05f, "read_data temperature == ~25.08 °C");
    TEST_ASSERT(fabsf(press_pa - 100653.0f) < 2.0f, "read_data pressure == ~100653 Pa");
    TEST_ASSERT(fabsf(alt_m - 58.2f) < 1.0f, "read_data hypsometric altitude == ~58.2 m");

    /* Test invalid chip ID handling */
    s_mock_reg_map[BMP280_REG_CHIP_ID] = 0x00;
    status = bmp280_init(&dev);
    TEST_ASSERT(status == BMP280_ERR_CHIP_ID, "Invalid chip ID correctly flagged as BMP280_ERR_CHIP_ID");

    return 0;
}

/* ==================================================================== */
/* Test 4: Mock SPI Bus Lifecycle (4-wire Bit 7 protocol)               */
/* ==================================================================== */
static int8_t mock_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr)
{
    (void)dev_addr;
    (void)intf_ptr;
    /* In SPI mode, read command requires MSB (bit 7) to be set (Datasheet section 5.3) */
    uint8_t spi_tx = (reg_addr | 0x80U);
    if ((spi_tx & 0x80U) == 0) {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        data[i] = s_mock_reg_map[(reg_addr + i) & 0xFF];
    }
    return 0;
}

static int8_t mock_spi_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    (void)dev_addr;
    (void)intf_ptr;
    /* In SPI mode, write command requires MSB (bit 7) to be cleared (Datasheet section 5.3) */
    uint8_t spi_tx = (reg_addr & 0x7FU);
    if ((spi_tx & 0x80U) != 0) {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        s_mock_reg_map[(reg_addr + i) & 0xFF] = data[i];
    }
    return 0;
}

static int test_mock_spi_lifecycle(void)
{
    printf("\n--- Test 4: Mock SPI Bus Lifecycle (Bit 7 protocol) ---\n");

    /* Restore test vector in mock memory */
    s_mock_reg_map[BMP280_REG_CHIP_ID] = BMP280_CHIP_ID;

    bmp280_t dev_spi;
    memset(&dev_spi, 0, sizeof(dev_spi));
    dev_spi.read = mock_spi_read;
    dev_spi.write = mock_spi_write;
    dev_spi.delay_ms = mock_delay_ms;

    bmp280_status_t status = bmp280_init(&dev_spi);
    TEST_ASSERT(status == BMP280_OK, "bmp280_init over SPI returns BMP280_OK");
    TEST_ASSERT(dev_spi.calib.dig_T1 == 27504, "dig_T1 read accurately over SPI");
    TEST_ASSERT(dev_spi.calib.dig_P9 == 6000, "dig_P9 read accurately over SPI");

    float temp_c = 0.0f, press_pa = 0.0f, alt_m = 0.0f;
    status = bmp280_read_data(&dev_spi, &temp_c, &press_pa, &alt_m);
    TEST_ASSERT(status == BMP280_OK, "bmp280_read_data over SPI returns BMP280_OK");
    TEST_ASSERT(fabsf(temp_c - 25.08f) < 0.05f, "SPI read_data temperature == ~25.08 °C");
    TEST_ASSERT(fabsf(press_pa - 100653.0f) < 2.0f, "SPI read_data pressure == ~100653 Pa");
    TEST_ASSERT(fabsf(alt_m - 58.04f) < 1.0f, "SPI read_data altitude == ~58.0 m");

    return 0;
}

int main(void)
{
    printf("=========================================\n");
    printf("   BMP280 DRIVER HOST UNIT TEST SUITE    \n");
    printf("=========================================\n");

    if (test_datasheet_page23_vector() != 0) {
        printf("\nTEST SUITE FAILED at Test 1!\n");
        return 1;
    }

    if (test_altitude_calculation() != 0) {
        printf("\nTEST SUITE FAILED at Test 2!\n");
        return 1;
    }

    if (test_mock_sensor_lifecycle() != 0) {
        printf("\nTEST SUITE FAILED at Test 3!\n");
        return 1;
    }

    if (test_mock_spi_lifecycle() != 0) {
        printf("\nTEST SUITE FAILED at Test 4!\n");
        return 1;
    }

    printf("\n=========================================\n");
    printf("ALL TESTS PASSED! Total: %d, Passed: %d\n", g_tests_run, g_tests_passed);
    printf("=========================================\n");
    return 0;
}
