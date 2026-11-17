/**
 ******************************************************************************
 * @file    test_speculative_edge_cases.c
 * @author  Staff SDET & Quality Gatekeeper
 * @brief   Adversarial and Speculative Edge-Case Test Suite for Bosch BMP280 Driver.
 *          Covers:
 *          - Bosch datasheet test vector (page 23: UT=519888, UP=415148)
 *          - Division by zero boundary conditions (var1 == 0 in 64-bit, 32-bit & float)
 *          - Sensor skip raw values (UT=0x80000, UP=0x80000)
 *          - Barometric altitude international hypsometric formula invariants
 *          - Calibration byte decoding & sign-extension (signed vs unsigned 16-bit)
 *          - 20-bit raw ADC unpack alignment
 *          - Extreme temperature & pressure boundaries (BVA: 0, 0xFFFFF, min/max operating)
 *          - Negative paths & fault injection (NULL pointers, chip ID mismatch, I2C bus error)
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "bmp280.h"

/* ==================================================================== */
/* Test Harness Framework & Counters                                    */
/* ==================================================================== */
static uint32_t s_total_tests = 0;
static uint32_t s_passed_tests = 0;
static uint32_t s_failed_tests = 0;

#define TEST_ASSERT(cond, msg, ...) do { \
    s_total_tests++; \
    if (cond) { \
        s_passed_tests++; \
    } else { \
        s_failed_tests++; \
        fprintf(stderr, "[FAIL] %s:%d: " msg "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    } \
} while (0)

#define TEST_ASSERT_FLOAT_NEAR(actual, expected, tol, msg) do { \
    float _diff = fabsf((actual) - (expected)); \
    TEST_ASSERT(_diff <= (tol), "%s: expected %.4f, got %.4f (diff: %.4f, tol: %.4f)", \
                msg, (float)(expected), (float)(actual), _diff, (float)(tol)); \
} while (0)

/* ==================================================================== */
/* Bosch Datasheet Reference Calibration Parameters (Chapter 8.1 / p.23) */
/* ==================================================================== */
static void setup_datasheet_calib(bmp280_t *dev)
{
    memset(dev, 0, sizeof(bmp280_t));
    dev->dev_addr = BMP280_I2C_ADDR_PRIM;

    dev->calib.dig_T1 = 27504;
    dev->calib.dig_T2 = 26435;
    dev->calib.dig_T3 = -1000;

    dev->calib.dig_P1 = 36477;
    dev->calib.dig_P2 = -10685;
    dev->calib.dig_P3 = 3024;
    dev->calib.dig_P4 = 2855;
    dev->calib.dig_P5 = 140;
    dev->calib.dig_P6 = -7;
    dev->calib.dig_P7 = 15500;
    dev->calib.dig_P8 = -14600;
    dev->calib.dig_P9 = 6000;
}

/* ==================================================================== */
/* Mock I2C Bus Infrastructure for Hardware Abstraction Testing         */
/* ==================================================================== */
typedef struct {
    uint8_t registers[256];
    int8_t  force_read_err;
    int8_t  force_write_err;
    uint8_t last_reg_written;
    uint8_t last_data_written[16];
    uint16_t last_len_written;
} mock_i2c_bus_t;

static int8_t mock_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr)
{
    (void)dev_addr;
    mock_i2c_bus_t *bus = (mock_i2c_bus_t *)intf_ptr;
    if (!bus || bus->force_read_err) {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        data[i] = bus->registers[(reg_addr + i) & 0xFF];
    }
    return 0;
}

static int8_t mock_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    (void)dev_addr;
    mock_i2c_bus_t *bus = (mock_i2c_bus_t *)intf_ptr;
    if (!bus || bus->force_write_err) {
        return -1;
    }
    bus->last_reg_written = reg_addr;
    bus->last_len_written = len;
    for (uint16_t i = 0; i < len; i++) {
        bus->registers[(reg_addr + i) & 0xFF] = data[i];
        if (i < sizeof(bus->last_data_written)) {
            bus->last_data_written[i] = data[i];
        }
    }
    return 0;
}

static void mock_delay_ms(uint32_t ms)
{
    (void)ms;
}

/* ==================================================================== */
/* TEST 1: Bosch Datasheet Reference Test Vector                        */
/* UT=519888, UP=415148 -> t_fine=128422, T=25.08 °C, P=100653 Pa       */
/* ==================================================================== */
static void test_bosch_datasheet_vector(void)
{
    printf("[RUN] Test 1: Bosch Datasheet Reference Vector...\n");
    bmp280_t dev;
    setup_datasheet_calib(&dev);

    const int32_t raw_ut = 519888;
    const int32_t raw_up = 415148;

    /* 1.1 Integer 32-bit Temperature */
    int32_t t_int = bmp280_compensate_T_int32(&dev, raw_ut);
    TEST_ASSERT(dev.t_fine == 128422, "t_fine must be 128422, got %d", dev.t_fine);
    TEST_ASSERT(t_int == 2508, "T_int32 must be 2508 (25.08 degC), got %d", t_int);

    /* 1.2 Integer 64-bit Pressure */
    uint32_t p_q24 = bmp280_compensate_P_int64(&dev, raw_up);
    uint32_t p_pa_from_64 = p_q24 >> 8;
    TEST_ASSERT(p_pa_from_64 == 100653, "P_int64 >> 8 must be 100653 Pa, got %u (Q24.8=%u)", p_pa_from_64, p_q24);

    /* 1.3 Integer 32-bit Pressure */
    uint32_t p_int32 = bmp280_compensate_P_int32(&dev, raw_up);
    /* 32-bit fixed point formula allows slight truncation rounding (100653..100656) */
    TEST_ASSERT(p_int32 >= 100650 && p_int32 <= 100660,
                "P_int32 must be ~100653 Pa (tolerance +/-7), got %u", p_int32);

    /* 1.4 Floating Point Temperature */
    dev.t_fine = 0;
    float t_flt = bmp280_compensate_T_float(&dev, raw_ut);
    TEST_ASSERT_FLOAT_NEAR(t_flt, 25.0825f, 0.05f, "T_float near 25.08 °C");
    TEST_ASSERT(dev.t_fine >= 128400 && dev.t_fine <= 128450,
                "t_fine updated by float temp to ~128422, got %d", dev.t_fine);

    /* 1.5 Floating Point Pressure */
    float p_flt = bmp280_compensate_P_float(&dev, raw_up);
    TEST_ASSERT_FLOAT_NEAR(p_flt, 100653.27f, 1.0f, "P_float near 100653 Pa");
}

/* ==================================================================== */
/* TEST 2: Division-by-Zero Protection in Pressure Formulas (var1 == 0)  */
/* ==================================================================== */
static void test_pressure_division_by_zero(void)
{
    printf("[RUN] Test 2: Pressure Division-by-Zero (var1 == 0 guard)...\n");
    bmp280_t dev;
    setup_datasheet_calib(&dev);

    /* Setting dig_P1 to 0 forces var1 to 0 in both 64-bit and 32-bit Bosch formulas */
    dev.calib.dig_P1 = 0;
    dev.t_fine = 128422;

    /* 2.1 64-bit formula: if var1 == 0, must safely return 0 without SIGFPE */
    uint32_t p64 = bmp280_compensate_P_int64(&dev, 415148);
    TEST_ASSERT(p64 == 0, "bmp280_compensate_P_int64 must return 0 when var1==0 (dig_P1=0), got %u", p64);

    /* 2.2 32-bit formula: if var1 == 0, must safely return 0 without SIGFPE */
    uint32_t p32 = bmp280_compensate_P_int32(&dev, 415148);
    TEST_ASSERT(p32 == 0, "bmp280_compensate_P_int32 must return 0 when var1==0 (dig_P1=0), got %u", p32);

    /* 2.3 Float formula: if var1 == 0.0f, must return 0.0f without divide-by-zero exception */
    float pflt = bmp280_compensate_P_float(&dev, 415148);
    TEST_ASSERT(pflt == 0.0f, "bmp280_compensate_P_float must return 0.0f when var1==0 (dig_P1=0), got %.2f", pflt);

    /* 2.4 NULL pointer safety in compensation functions */
    TEST_ASSERT(bmp280_compensate_T_int32(NULL, 519888) == 0, "Compensate T_int32(NULL) returns 0");
    TEST_ASSERT(bmp280_compensate_P_int64(NULL, 415148) == 0, "Compensate P_int64(NULL) returns 0");
    TEST_ASSERT(bmp280_compensate_P_int32(NULL, 415148) == 0, "Compensate P_int32(NULL) returns 0");
    TEST_ASSERT(bmp280_compensate_T_float(NULL, 519888) == 0.0f, "Compensate T_float(NULL) returns 0.0f");
    TEST_ASSERT(bmp280_compensate_P_float(NULL, 415148) == 0.0f, "Compensate P_float(NULL) returns 0.0f");
}

/* ==================================================================== */
/* TEST 3: Sensor Skip Values (UT=0x80000, UP=0x80000)                   */
/* ==================================================================== */
static void test_sensor_skip_values(void)
{
    printf("[RUN] Test 3: Sensor Skip Values (0x80000)...\n");
    bmp280_t dev;
    setup_datasheet_calib(&dev);

    const int32_t skip_val = 0x80000; /* 524288: BMP280 output register when skipped/disabled */

    /* Ensure compensation does not crash or cause arithmetic faults with 0x80000 */
    int32_t t_skip = bmp280_compensate_T_int32(&dev, skip_val);
    TEST_ASSERT(dev.t_fine != 0, "t_fine computed without crash on UT=0x80000, got %d", dev.t_fine);
    (void)t_skip;

    uint32_t p64_skip = bmp280_compensate_P_int64(&dev, skip_val);
    TEST_ASSERT(p64_skip != 0xFFFFFFFFU, "P_int64 completed without unhandled overflow on UP=0x80000");

    uint32_t p32_skip = bmp280_compensate_P_int32(&dev, skip_val);
    (void)p32_skip;

    float t_flt_skip = bmp280_compensate_T_float(&dev, skip_val);
    TEST_ASSERT(!isnan(t_flt_skip) && !isinf(t_flt_skip), "T_float is finite on UT=0x80000");

    float p_flt_skip = bmp280_compensate_P_float(&dev, skip_val);
    TEST_ASSERT(!isnan(p_flt_skip) && !isinf(p_flt_skip), "P_float is finite on UP=0x80000");
}

/* ==================================================================== */
/* TEST 4: Barometric Altitude International Hypsometric Formula         */
/* ==================================================================== */
static void test_barometric_altitude_invariants(void)
{
    printf("[RUN] Test 4: Barometric Altitude Invariants & Edge Cases...\n");

    /* Standard sea-level: 1013.25 hPa -> altitude ~ 0.0m (+/- 0.1m) */
    float alt_sea = bmp280_calculate_altitude(1013.25f, 1013.25f);
    TEST_ASSERT_FLOAT_NEAR(alt_sea, 0.0f, 0.1f, "Altitude at P=1013.25 hPa with P0=1013.25 hPa");

    /* 850.00 hPa -> altitude ~ 1457m (+/- 5m) */
    float alt_850 = bmp280_calculate_altitude(850.0f, 1013.25f);
    TEST_ASSERT_FLOAT_NEAR(alt_850, 1457.15f, 5.0f, "Altitude at P=850.0 hPa with P0=1013.25 hPa");

    /* Default fallback sea level when P0 <= 0 */
    float alt_p0_zero = bmp280_calculate_altitude(1013.25f, 0.0f);
    TEST_ASSERT_FLOAT_NEAR(alt_p0_zero, 0.0f, 0.1f, "Altitude with sea_level_hpa=0.0f falls back to standard P0");

    float alt_p0_neg = bmp280_calculate_altitude(1013.25f, -100.0f);
    TEST_ASSERT_FLOAT_NEAR(alt_p0_neg, 0.0f, 0.1f, "Altitude with negative sea_level_hpa falls back to standard P0");

    /* Boundary: Non-positive pressure -> returns 0.0f */
    float alt_press_zero = bmp280_calculate_altitude(0.0f, 1013.25f);
    TEST_ASSERT(alt_press_zero == 0.0f, "Altitude with press_hpa=0.0f must return 0.0f, got %.2f", alt_press_zero);

    float alt_press_neg = bmp280_calculate_altitude(-50.0f, 1013.25f);
    TEST_ASSERT(alt_press_neg == 0.0f, "Altitude with negative press_hpa must return 0.0f, got %.2f", alt_press_neg);

    /* Extreme high altitude (Troposphere edge): 300.0 hPa (~9164m) */
    float alt_300 = bmp280_calculate_altitude(300.0f, 1013.25f);
    TEST_ASSERT_FLOAT_NEAR(alt_300, 9164.0f, 50.0f, "Altitude at 300 hPa near 9164m");

    /* Below sea level (Dead Sea shore): 1080.0 hPa (~ -540m) */
    float alt_below = bmp280_calculate_altitude(1080.0f, 1013.25f);
    TEST_ASSERT_FLOAT_NEAR(alt_below, -540.0f, 25.0f, "Altitude at 1080 hPa below sea level");

    /* Hypsometric altitude with temperature compensation */
    float alt_hyp_sea = bmp280_calculate_altitude_hypsometric(1013.25f, 20.0f, 1013.25f);
    TEST_ASSERT_FLOAT_NEAR(alt_hyp_sea, 0.0f, 0.1f, "Hypsometric altitude at P=P0 is 0.0m");

    /* At T=15°C, hypsometric matches standard atmosphere (~1457m at 850 hPa) */
    float alt_hyp_850_15c = bmp280_calculate_altitude_hypsometric(850.0f, 15.0f, 1013.25f);
    TEST_ASSERT_FLOAT_NEAR(alt_hyp_850_15c, 1457.15f, 5.0f, "Hypsometric at 15°C matches standard ISA");

    /* At T=35°C (hot summer day), warmer air expands -> column height increases to ~1558m */
    float alt_hyp_850_35c = bmp280_calculate_altitude_hypsometric(850.0f, 35.0f, 1013.25f);
    TEST_ASSERT(alt_hyp_850_35c > alt_hyp_850_15c, "Warmer air must yield higher geometric altitude column");

    /* Unphysical low temp fallback (-300°C) falls back to 15°C */
    float alt_hyp_unphysical = bmp280_calculate_altitude_hypsometric(850.0f, -300.0f, 1013.25f);
    TEST_ASSERT_FLOAT_NEAR(alt_hyp_unphysical, alt_hyp_850_15c, 0.1f, "Unphysical < -273.15C falls back to 15C");
}

/* ==================================================================== */
/* TEST 5: Calibration Byte Decoding & Sign Extension                   */
/* 24-byte burst read from registers 0x88..0x9F                         */
/* ==================================================================== */
static void test_calibration_decoding(void)
{
    printf("[RUN] Test 5: Calibration Decoding & Sign Extension...\n");
    mock_i2c_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    /* Populate registers with Chip ID */
    bus.registers[BMP280_REG_CHIP_ID] = BMP280_CHIP_ID;
    bus.registers[BMP280_REG_STATUS] = 0x00; /* Ready (not measuring, not updating) */

    /*
     * Bosch reference calibration parameters:
     * dig_T1: 27504 (0x6B70) -> LSB=0x70, MSB=0x6B (uint16)
     * dig_T2: 26435 (0x6743) -> LSB=0x43, MSB=0x67 (int16)
     * dig_T3: -1000 (0xFC18) -> LSB=0x18, MSB=0xFC (int16)
     * dig_P1: 36477 (0x8E7D) -> LSB=0x7D, MSB=0x8E (uint16)
     * dig_P2: -10685 (0xD643)-> LSB=0x43, MSB=0xD6 (int16)
     * dig_P3: 3024  (0x0BD0) -> LSB=0xD0, MSB=0x0B (int16)
     * dig_P4: 2855  (0x0B27) -> LSB=0x27, MSB=0x0B (int16)
     * dig_P5: 140   (0x008C) -> LSB=0x8C, MSB=0x00 (int16)
     * dig_P6: -7    (0xFFF9) -> LSB=0xF9, MSB=0xFF (int16)
     * dig_P7: 15500 (0x3C8C) -> LSB=0x8C, MSB=0x3C (int16)
     * dig_P8: -14600 (0xC6F8)-> LSB=0xF8, MSB=0xC6 (int16)
     * dig_P9: 6000  (0x1770) -> LSB=0x70, MSB=0x17 (int16)
     */
    static const uint8_t calib_bytes[24] = {
        0x70, 0x6B,  /* dig_T1 = 27504 */
        0x43, 0x67,  /* dig_T2 = 26435 */
        0x18, 0xFC,  /* dig_T3 = -1000 */
        0x7D, 0x8E,  /* dig_P1 = 36477 */
        0x43, 0xD6,  /* dig_P2 = -10685 */
        0xD0, 0x0B,  /* dig_P3 = 3024 */
        0x27, 0x0B,  /* dig_P4 = 2855 */
        0x8C, 0x00,  /* dig_P5 = 140 */
        0xF9, 0xFF,  /* dig_P6 = -7 */
        0x8C, 0x3C,  /* dig_P7 = 15500 */
        0xF8, 0xC6,  /* dig_P8 = -14600 */
        0x70, 0x17   /* dig_P9 = 6000 */
    };
    memcpy(&bus.registers[BMP280_REG_CALIB_START], calib_bytes, 24);

    bmp280_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.dev_addr = BMP280_I2C_ADDR_PRIM;
    dev.read = mock_i2c_read;
    dev.write = mock_i2c_write;
    dev.delay_ms = mock_delay_ms;
    dev.intf_ptr = &bus;

    bmp280_status_t status = bmp280_init(&dev);
    TEST_ASSERT(status == BMP280_OK, "bmp280_init succeeded with mock bus");

    /* Verify exact decoding */
    TEST_ASSERT(dev.calib.dig_T1 == 27504, "dig_T1 decoded: expected 27504, got %u", dev.calib.dig_T1);
    TEST_ASSERT(dev.calib.dig_T2 == 26435, "dig_T2 decoded: expected 26435, got %d", dev.calib.dig_T2);
    TEST_ASSERT(dev.calib.dig_T3 == -1000, "dig_T3 decoded: expected -1000, got %d", dev.calib.dig_T3);

    /* Verify signed vs unsigned distinction */
    TEST_ASSERT(dev.calib.dig_P1 == 36477, "dig_P1 decoded: expected 36477, got %u", dev.calib.dig_P1);
    TEST_ASSERT(dev.calib.dig_P2 == -10685, "dig_P2 decoded: expected -10685, got %d", dev.calib.dig_P2);
    TEST_ASSERT(dev.calib.dig_P3 == 3024, "dig_P3 decoded: expected 3024, got %d", dev.calib.dig_P3);
    TEST_ASSERT(dev.calib.dig_P4 == 2855, "dig_P4 decoded: expected 2855, got %d", dev.calib.dig_P4);
    TEST_ASSERT(dev.calib.dig_P5 == 140, "dig_P5 decoded: expected 140, got %d", dev.calib.dig_P5);
    TEST_ASSERT(dev.calib.dig_P6 == -7, "dig_P6 decoded: expected -7, got %d", dev.calib.dig_P6);
    TEST_ASSERT(dev.calib.dig_P7 == 15500, "dig_P7 decoded: expected 15500, got %d", dev.calib.dig_P7);
    TEST_ASSERT(dev.calib.dig_P8 == -14600, "dig_P8 decoded: expected -14600, got %d", dev.calib.dig_P8);
    TEST_ASSERT(dev.calib.dig_P9 == 6000, "dig_P9 decoded: expected 6000, got %d", dev.calib.dig_P9);
}

/* ==================================================================== */
/* TEST 6: 20-bit Raw ADC Unpacking Bit Alignment                       */
/* Registers 0xF7..0xFC                                                 */
/* ==================================================================== */
static void test_raw_adc_unpacking(void)
{
    printf("[RUN] Test 6: 20-bit Raw ADC Unpack Alignment...\n");
    mock_i2c_bus_t bus;
    memset(&bus, 0, sizeof(bus));

    /*
     * For UT=519888 (0x7EFD0 or 0x7EF00):
     * 519888 = 0x7EFD0 (20-bit: [19:12]=0x7E, [11:4]=0xF0, [3:0]=0x00)
     * For UP=415148:
     * 415148 = 0x655AC (20-bit: [19:12]=0x65, [11:4]=0x5A, [3:0]=0xC0 in byte)
     */
    const uint32_t expected_up = 415148;
    const uint32_t expected_ut = 519888;

    bus.registers[BMP280_REG_PRESS_MSB]  = (uint8_t)((expected_up >> 12) & 0xFF);
    bus.registers[BMP280_REG_PRESS_LSB]  = (uint8_t)((expected_up >> 4) & 0xFF);
    bus.registers[BMP280_REG_PRESS_XLSB] = (uint8_t)((expected_up & 0x0F) << 4);

    bus.registers[BMP280_REG_TEMP_MSB]   = (uint8_t)((expected_ut >> 12) & 0xFF);
    bus.registers[BMP280_REG_TEMP_LSB]   = (uint8_t)((expected_ut >> 4) & 0xFF);
    bus.registers[BMP280_REG_TEMP_XLSB]  = (uint8_t)((expected_ut & 0x0F) << 4);

    bmp280_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.dev_addr = BMP280_I2C_ADDR_PRIM;
    dev.read = mock_i2c_read;
    dev.write = mock_i2c_write;
    dev.intf_ptr = &bus;

    int32_t raw_temp = 0, raw_press = 0;
    bmp280_status_t status = bmp280_read_raw(&dev, &raw_temp, &raw_press);

    TEST_ASSERT(status == BMP280_OK, "bmp280_read_raw returned BMP280_OK");
    TEST_ASSERT(raw_press == (int32_t)expected_up, "Raw pressure unpack: expected %u, got %d", expected_up, raw_press);
    TEST_ASSERT(raw_temp == (int32_t)expected_ut, "Raw temperature unpack: expected %u, got %d", expected_ut, raw_temp);
}

/* ==================================================================== */
/* TEST 7: Boundary Value Analysis (BVA) on Extreme Raw Values           */
/* ==================================================================== */
static void test_boundary_extreme_values(void)
{
    printf("[RUN] Test 7: Boundary Value Analysis (BVA) Extreme Values...\n");
    bmp280_t dev;
    setup_datasheet_calib(&dev);

    /* 7.1 Zero raw temperature */
    int32_t t_zero = bmp280_compensate_T_int32(&dev, 0);
    (void)t_zero;
    TEST_ASSERT(dev.t_fine != 0, "t_fine computed on adc_T=0 without fault");

    /* 7.2 Max 20-bit raw temperature (0xFFFFF = 1048575) */
    int32_t t_max = bmp280_compensate_T_int32(&dev, 0xFFFFF);
    (void)t_max;
    TEST_ASSERT(dev.t_fine != 0, "t_fine computed on adc_T=0xFFFFF without fault");

    /* 7.3 Zero raw pressure */
    dev.t_fine = 128422;
    uint32_t p64_zero = bmp280_compensate_P_int64(&dev, 0);
    uint32_t p32_zero = bmp280_compensate_P_int32(&dev, 0);
    float pflt_zero = bmp280_compensate_P_float(&dev, 0);
    (void)p64_zero; (void)p32_zero;
    TEST_ASSERT(!isnan(pflt_zero), "P_float is finite on adc_P=0");

    /* 7.4 Max 20-bit raw pressure (0xFFFFF) */
    uint32_t p64_max = bmp280_compensate_P_int64(&dev, 0xFFFFF);
    uint32_t p32_max = bmp280_compensate_P_int32(&dev, 0xFFFFF);
    float pflt_max = bmp280_compensate_P_float(&dev, 0xFFFFF);
    (void)p64_max; (void)p32_max;
    TEST_ASSERT(!isnan(pflt_max), "P_float is finite on adc_P=0xFFFFF");
}

/* ==================================================================== */
/* TEST 8: Negative Path & Fault Injection Testing                      */
/* ==================================================================== */
static void test_negative_paths_and_fault_injection(void)
{
    printf("[RUN] Test 8: Negative Paths & Fault Injection...\n");

    /* 8.1 NULL handle validation across all public API calls */
    TEST_ASSERT(bmp280_init(NULL) == BMP280_ERR_INVALID_PARAM, "init(NULL) rejected");
    TEST_ASSERT(bmp280_reset(NULL) == BMP280_ERR_INVALID_PARAM, "reset(NULL) rejected");
    TEST_ASSERT(bmp280_set_config(NULL, NULL) == BMP280_ERR_INVALID_PARAM, "set_config(NULL, NULL) rejected");

    bool busy = false;
    TEST_ASSERT(bmp280_is_measuring(NULL, &busy) == BMP280_ERR_INVALID_PARAM, "is_measuring(NULL) rejected");

    int32_t rt = 0, rp = 0;
    TEST_ASSERT(bmp280_read_raw(NULL, &rt, &rp) == BMP280_ERR_INVALID_PARAM, "read_raw(NULL) rejected");
    TEST_ASSERT(bmp280_read_data(NULL, NULL, NULL, NULL) == BMP280_ERR_INVALID_PARAM, "read_data(NULL) rejected");

    /* 8.2 Missing callbacks */
    bmp280_t invalid_dev;
    memset(&invalid_dev, 0, sizeof(invalid_dev));
    TEST_ASSERT(bmp280_init(&invalid_dev) == BMP280_ERR_INVALID_PARAM, "init without read/write rejected");

    /* 8.3 Chip ID mismatch injection */
    mock_i2c_bus_t bus;
    memset(&bus, 0, sizeof(bus));
    bus.registers[BMP280_REG_CHIP_ID] = 0x55; /* Invalid chip ID (expected 0x58) */

    bmp280_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.read = mock_i2c_read;
    dev.write = mock_i2c_write;
    dev.intf_ptr = &bus;

    TEST_ASSERT(bmp280_init(&dev) == BMP280_ERR_CHIP_ID, "init with chip ID 0x55 returns ERR_CHIP_ID");

    /* 8.4 I2C read failure injection */
    bus.registers[BMP280_REG_CHIP_ID] = BMP280_CHIP_ID;
    bus.force_read_err = -1;
    TEST_ASSERT(bmp280_init(&dev) == BMP280_ERR_COMM, "init with I2C bus read error returns ERR_COMM");

    /* 8.5 I2C write failure injection */
    bus.force_read_err = 0;
    bus.force_write_err = -1;
    TEST_ASSERT(bmp280_reset(&dev) == BMP280_ERR_COMM, "reset with I2C bus write error returns ERR_COMM");
}

/* ==================================================================== */
/* Main Test Runner Entrypoint                                          */
/* ==================================================================== */
int main(void)
{
    printf("=================================================================\n");
    printf("BMP280 SPECULATIVE & ADVERSARIAL EDGE-CASE TEST SUITE (SDET GATE)\n");
    printf("=================================================================\n");

    test_bosch_datasheet_vector();
    test_pressure_division_by_zero();
    test_sensor_skip_values();
    test_barometric_altitude_invariants();
    test_calibration_decoding();
    test_raw_adc_unpacking();
    test_boundary_extreme_values();
    test_negative_paths_and_fault_injection();

    printf("=================================================================\n");
    printf("SUMMARY: %u ran | %u passed | %u failed\n", s_total_tests, s_passed_tests, s_failed_tests);
    printf("=================================================================\n");

    if (s_failed_tests > 0) {
        printf("[RESULT] GATE STATUS: REJECTED (Failures detected)\n");
        return 1;
    }

    printf("[RESULT] GATE STATUS: APPROVED (All deterministic assertions passed)\n");
    return 0;
}
