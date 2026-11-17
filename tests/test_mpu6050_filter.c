/**
 ******************************************************************************
 * @file    test_mpu6050_filter.c
 * @brief   Host unit tests for MPU-6050 driver, Complementary Filter, and
 *          shared-I2C-bus coexistence with Bosch BMP280 (I2C spec checks).
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "Mpu6050.h"
#include "ComplementaryFilter.h"
#include "bmp280.h"

static int g_tests_run = 0;
static int g_tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        fprintf(stderr, "  [FAIL] %s (Line %d)\n", msg, __LINE__); \
        return 1; \
    } \
} while (0)

typedef struct {
    uint8_t bmp_regs[256];
    uint8_t mpu_regs[256];
    bool    fail_mpu_write_on_reg;
    uint8_t fail_write_target_reg;
    bool    fail_mpu_read;
    uint8_t bmp_write_history_regs[16];
    uint8_t bmp_write_history_vals[16];
    uint8_t bmp_write_count;
} shared_i2c_bus_t;

static int8_t shared_bmp_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr)
{
    shared_i2c_bus_t *bus = (shared_i2c_bus_t *)intf_ptr;
    if (!bus || dev_addr != BMP280_I2C_ADDR_PRIM || !data || len == 0U) {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        data[i] = bus->bmp_regs[(reg_addr + i) & 0xFFU];
    }
    return 0;
}

static int8_t shared_bmp_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    shared_i2c_bus_t *bus = (shared_i2c_bus_t *)intf_ptr;
    if (!bus || dev_addr != BMP280_I2C_ADDR_PRIM || !data || len == 0U) {
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        uint8_t r = (uint8_t)(reg_addr + i);
        bus->bmp_regs[r] = data[i];
        if (bus->bmp_write_count < 16U) {
            bus->bmp_write_history_regs[bus->bmp_write_count] = r;
            bus->bmp_write_history_vals[bus->bmp_write_count] = data[i];
            bus->bmp_write_count++;
        }
    }
    return 0;
}

static int8_t shared_mpu_read(u8 devAddr7Bit, u8 reg, u8 *pBuffer, u16 len, void *pCtx)
{
    shared_i2c_bus_t *bus = (shared_i2c_bus_t *)pCtx;
    if (!bus || bus->fail_mpu_read || devAddr7Bit != dMPU6050_I2C_ADDR_7BIT || !pBuffer || len == 0U) {
        return -1;
    }
    for (u16 i = 0; i < len; i++) {
        pBuffer[i] = bus->mpu_regs[(reg + i) & 0xFFU];
    }
    return 0;
}

static int8_t shared_mpu_write(u8 devAddr7Bit, u8 reg, const u8 *pData, u16 len, void *pCtx)
{
    shared_i2c_bus_t *bus = (shared_i2c_bus_t *)pCtx;
    if (!bus || devAddr7Bit != dMPU6050_I2C_ADDR_7BIT || !pData || len == 0U) {
        return -1;
    }
    if (bus->fail_mpu_write_on_reg && reg == bus->fail_write_target_reg) {
        return -1;
    }
    for (u16 i = 0; i < len; i++) {
        bus->mpu_regs[(reg + i) & 0xFFU] = pData[i];
    }
    return 0;
}

static void noop_delay(uint32_t ms)
{
    (void)ms;
}

static void init_shared_bus(shared_i2c_bus_t *bus)
{
    memset(bus, 0, sizeof(*bus));

    /* BMP280 mock registers (Datasheet page 23 vector) */
    bus->bmp_regs[BMP280_REG_CHIP_ID] = BMP280_CHIP_ID;
    bus->bmp_regs[BMP280_REG_STATUS]  = 0x00U;
    static const uint8_t calib_bytes[24] = {
        0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC,
        0x7D, 0x8E, 0x43, 0xD6, 0xD0, 0x0B,
        0x27, 0x0B, 0x8C, 0x00, 0xF9, 0xFF,
        0x8C, 0x3C, 0xF8, 0xC6, 0x70, 0x17
    };
    memcpy(&bus->bmp_regs[BMP280_REG_CALIB_START], calib_bytes, 24U);

    bus->bmp_regs[BMP280_REG_PRESS_MSB]  = (uint8_t)((415148 >> 12) & 0xFFU);
    bus->bmp_regs[BMP280_REG_PRESS_LSB]  = (uint8_t)((415148 >> 4) & 0xFFU);
    bus->bmp_regs[BMP280_REG_PRESS_XLSB] = (uint8_t)((415148 << 4) & 0xF0U);
    bus->bmp_regs[BMP280_REG_TEMP_MSB]   = (uint8_t)((519888 >> 12) & 0xFFU);
    bus->bmp_regs[BMP280_REG_TEMP_LSB]   = (uint8_t)((519888 >> 4) & 0xFFU);
    bus->bmp_regs[BMP280_REG_TEMP_XLSB]  = (uint8_t)((519888 << 4) & 0xF0U);

    /* MPU-6050 mock registers */
    bus->mpu_regs[dMPU6050_REG_WHO_AM_I] = dMPU6050_WHO_AM_I_VALUE;
}

static int test_mpu6050_init_and_errors(void)
{
    printf("--- Test 1: MPU-6050 Init & Fault Injection ---\n");
    shared_i2c_bus_t bus;
    init_shared_bus(&bus);

    /* Invalid params */
    TEST_ASSERT(Mpu6050_InitWithBus(NULL, shared_mpu_write, &bus, dMPU6050_I2C_ADDR_7BIT) == false,
                "Reject NULL readFp");
    TEST_ASSERT(Mpu6050_InitWithBus(shared_mpu_read, NULL, &bus, dMPU6050_I2C_ADDR_7BIT) == false,
                "Reject NULL writeFp");
    TEST_ASSERT(Mpu6050_InitWithBus(shared_mpu_read, shared_mpu_write, &bus, 0x70U) == false,
                "Reject invalid 7-bit I2C address");

    /* Wrong WHO_AM_I */
    bus.mpu_regs[dMPU6050_REG_WHO_AM_I] = 0x55U;
    TEST_ASSERT(Mpu6050_InitWithBus(shared_mpu_read, shared_mpu_write, &bus, dMPU6050_I2C_ADDR_7BIT) == false,
                "Reject wrong WHO_AM_I");

    /* Write failure mid-init */
    bus.mpu_regs[dMPU6050_REG_WHO_AM_I] = dMPU6050_WHO_AM_I_VALUE;
    bus.fail_mpu_write_on_reg = true;
    bus.fail_write_target_reg = dMPU6050_REG_SMPLRT_DIV;
    TEST_ASSERT(Mpu6050_InitWithBus(shared_mpu_read, shared_mpu_write, &bus, dMPU6050_I2C_ADDR_7BIT) == false,
                "Abort init on register write error");
    bus.fail_mpu_write_on_reg = false;

    /* Valid init */
    TEST_ASSERT(Mpu6050_InitWithBus(shared_mpu_read, shared_mpu_write, &bus, dMPU6050_I2C_ADDR_7BIT) == true,
                "Valid MPU-6050 init returns true");
    TEST_ASSERT(bus.mpu_regs[dMPU6050_REG_PWR_MGMT_1] == 0x01U, "PWR_MGMT_1 set to PLL X (0x01)");
    TEST_ASSERT(bus.mpu_regs[dMPU6050_REG_SMPLRT_DIV] == 0x09U, "SMPLRT_DIV set to 100 Hz (0x09)");
    TEST_ASSERT(bus.mpu_regs[dMPU6050_REG_CONFIG] == 0x03U, "CONFIG DLPF set to 44 Hz (0x03)");
    TEST_ASSERT(bus.mpu_regs[dMPU6050_REG_GYRO_CONFIG] == 0x00U, "GYRO_CONFIG +/-250 dps");
    TEST_ASSERT(bus.mpu_regs[dMPU6050_REG_ACCEL_CONFIG] == 0x00U, "ACCEL_CONFIG +/-2g");

    TEST_ASSERT(Mpu6050_Sleep() == true, "Mpu6050_Sleep succeeds");
    TEST_ASSERT(bus.mpu_regs[dMPU6050_REG_PWR_MGMT_1] == 0x40U, "Sleep sets bit 6 (0x40)");
    TEST_ASSERT(Mpu6050_Wakeup() == true, "Mpu6050_Wakeup succeeds");
    TEST_ASSERT(bus.mpu_regs[dMPU6050_REG_PWR_MGMT_1] == 0x01U, "Wakeup restores PLL (0x01)");

    return 0;
}

static int test_mpu6050_read_and_scaling(void)
{
    printf("--- Test 2: MPU-6050 14-Byte Burst Read & Scaling ---\n");
    shared_i2c_bus_t bus;
    init_shared_bus(&bus);
    TEST_ASSERT(Mpu6050_InitWithBus(shared_mpu_read, shared_mpu_write, &bus, dMPU6050_I2C_ADDR_7BIT) == true,
                "Init before burst read");

    /* Accel: X=0, Y=-16384 (-1.0g), Z=+16384 (+1.0g) */
    /* Temp:  raw=0 -> 36.53 degC */
    /* Gyro:  X=+131 (+1.0 dps), Y=-262 (-2.0 dps), Z=+1310 (+10.0 dps) */
    const int16_t ax = 0, ay = -16384, az = 16384;
    const int16_t tp = 0;
    const int16_t gx = 131, gy = -262, gz = 1310;

    const int16_t vals[7] = {ax, ay, az, tp, gx, gy, gz};
    for (int i = 0; i < 7; i++) {
        bus.mpu_regs[dMPU6050_REG_ACCEL_XOUT_H + (i * 2)]     = (uint8_t)(((uint16_t)vals[i]) >> 8);
        bus.mpu_regs[dMPU6050_REG_ACCEL_XOUT_H + (i * 2) + 1] = (uint8_t)(((uint16_t)vals[i]) & 0xFFU);
    }

    mpu6050Data_t data;
    TEST_ASSERT(Mpu6050_ReadSensor(NULL) == false, "ReadSensor(NULL) rejected");
    TEST_ASSERT(Mpu6050_ReadSensor(&data) == true, "ReadSensor succeeds");
    TEST_ASSERT(fabsf(data.accelXG) < 1e-4f, "Accel X == 0.0 g");
    TEST_ASSERT(fabsf(data.accelYG + 1.0f) < 1e-4f, "Accel Y == -1.0 g");
    TEST_ASSERT(fabsf(data.accelZG - 1.0f) < 1e-4f, "Accel Z == +1.0 g");
    TEST_ASSERT(fabsf(data.tempC - 36.53f) < 1e-2f, "Temp == 36.53 degC");
    TEST_ASSERT(fabsf(data.gyroXDps - 1.0f) < 1e-4f, "Gyro X == +1.0 dps");
    TEST_ASSERT(fabsf(data.gyroYDps + 2.0f) < 1e-4f, "Gyro Y == -2.0 dps");
    TEST_ASSERT(fabsf(data.gyroZDps - 10.0f) < 1e-4f, "Gyro Z == +10.0 dps");

    bus.fail_mpu_read = true;
    TEST_ASSERT(Mpu6050_ReadSensor(&data) == false, "Read failure returns false");

    return 0;
}

static int test_complementary_filter(void)
{
    printf("--- Test 3: Complementary Filter Pose Fusion ---\n");
    pose_t pose;
    ComplementaryFilter_Init(&pose, 1.5f);
    TEST_ASSERT(fabsf(pose.alpha - 1.0f) < 1e-5f, "Alpha clamped high to 1.0");
    ComplementaryFilter_Init(&pose, dFILTER_DEFAULT_ALPHA);
    TEST_ASSERT(fabsf(pose.alpha - 0.98f) < 1e-5f, "Default alpha == 0.98");

    mpu6050Data_t imu;
    memset(&imu, 0, sizeof(imu));
    imu.accelZG = 1.0f;

    /* Flat stationary */
    ComplementaryFilter_Update(&pose, &imu, 0.01f);
    TEST_ASSERT(fabsf(pose.rollDeg) < 1e-3f && fabsf(pose.pitchDeg) < 1e-3f,
                "Flat stationary keeps 0 deg roll/pitch");

    /* Guard dt <= 0 */
    pose.yawDeg = 5.0f;
    imu.gyroZDps = 100.0f;
    ComplementaryFilter_Update(&pose, &imu, 0.0f);
    TEST_ASSERT(fabsf(pose.yawDeg - 5.0f) < 1e-5f, "Zero dtS ignored cleanly");

    /* Yaw integration and +90 deg Roll convergence */
    imu.accelYG = 1.0f;
    imu.accelZG = 0.0f;
    imu.gyroZDps = 45.0f;
    for (int i = 0; i < 300; i++) {
        ComplementaryFilter_Update(&pose, &imu, 0.01f);
    }
    TEST_ASSERT(fabsf(pose.rollDeg - 90.0f) < 0.5f, "Roll converges toward +90 deg");
    TEST_ASSERT(fabsf(pose.yawDeg - 140.0f) < 0.1f, "Yaw integrates gyroZ * dt (5 + 45*3 = 140 deg)");

    return 0;
}

static int test_shared_bus_bosch_i2c_coexistence(void)
{
    printf("--- Test 4: Shared I2C Bus Coexistence & Bosch Spec Compliance ---\n");
    shared_i2c_bus_t bus;
    init_shared_bus(&bus);

    bmp280_t bmp;
    memset(&bmp, 0, sizeof(bmp));
    bmp.dev_addr = BMP280_I2C_ADDR_PRIM;
    bmp.read = shared_bmp_read;
    bmp.write = shared_bmp_write;
    bmp.delay_ms = noop_delay;
    bmp.intf_ptr = &bus;

    TEST_ASSERT(bmp280_init(&bmp) == BMP280_OK, "BMP280 init succeeds on shared I2C bus");
    TEST_ASSERT(Mpu6050_InitWithBus(shared_mpu_read, shared_mpu_write, &bus, dMPU6050_I2C_ADDR_7BIT) == true,
                "MPU-6050 init succeeds on same shared I2C bus");

    /* Bosch Spec Section 4.3.5: spi3w_en (bit 0 of 0xF5) must be 0 in I2C mode */
    TEST_ASSERT((bus.bmp_regs[BMP280_REG_CONFIG] & 0x01U) == 0U,
                "BMP280 CONFIG bit 0 (spi3w_en) is 0 in I2C mode");

    /* Bosch Spec Section 4.3.4: reset -> sleep (0xF4=0x00) -> config (0xF5) -> ctrl_meas (0xF4) */
    TEST_ASSERT(bus.bmp_write_count == 4U, "BMP280 init writes exactly 4 registers");
    TEST_ASSERT(bus.bmp_write_history_regs[0] == BMP280_REG_RESET && bus.bmp_write_history_vals[0] == BMP280_RESET_VALUE,
                "Write 1: Soft Reset 0xE0 = 0xB6");
    TEST_ASSERT(bus.bmp_write_history_regs[1] == BMP280_REG_CTRL_MEAS && bus.bmp_write_history_vals[1] == 0x00U,
                "Write 2: Sleep mode before CONFIG write");
    TEST_ASSERT(bus.bmp_write_history_regs[2] == BMP280_REG_CONFIG,
                "Write 3: CONFIG register 0xF5");
    TEST_ASSERT(bus.bmp_write_history_regs[3] == BMP280_REG_CTRL_MEAS && (bus.bmp_write_history_vals[3] & 0x03U) == BMP280_MODE_NORMAL,
                "Write 4: Normal mode in CTRL_MEAS 0xF4");

    /* Verify invalid config parameters rejected */
    bmp280_config_t bad_cfg = bmp.config;
    bad_cfg.filter = (bmp280_filter_t)9;
    TEST_ASSERT(bmp280_set_config(&bmp, &bad_cfg) == BMP280_ERR_INVALID_PARAM,
                "Invalid filter value rejected");

    /* Verify NVM timeout error when im_update remains stuck */
    bus.bmp_regs[BMP280_REG_STATUS] = BMP280_STATUS_IM_UPDATE;
    TEST_ASSERT(bmp280_init(&bmp) == BMP280_ERR_TIMEOUT,
                "Stuck im_update triggers BMP280_ERR_TIMEOUT");

    return 0;
}

int main(void)
{
    printf("=============================================================\n");
    printf("   MPU-6050 + COMPLEMENTARY FILTER + SHARED I2C TEST SUITE   \n");
    printf("=============================================================\n");

    if (test_mpu6050_init_and_errors() != 0) return 1;
    if (test_mpu6050_read_and_scaling() != 0) return 1;
    if (test_complementary_filter() != 0) return 1;
    if (test_shared_bus_bosch_i2c_coexistence() != 0) return 1;

    printf("ALL MPU6050 & SHARED I2C TESTS PASSED! (%d/%d)\n", g_tests_passed, g_tests_run);
    return 0;
}
