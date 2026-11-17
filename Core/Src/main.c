/**
  ******************************************************************************
  * @file    main.c
  * @brief   NUCLEO-F767ZI Dual-Sensor Monitor: BMP280 (I2C) + MPU-6050 (I2C)
  *          with Complementary Filter pose estimation and non-blocking USART3
  *          IT telemetry (50 Hz IMU/Pose, 10 Hz BMP280).
  ******************************************************************************
  */

#include "main.h"
#include "bmp280.h"
#include "Mpu6050.h"
#include "ComplementaryFilter.h"
#include <stdio.h>
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#define IMU_SAMPLE_PERIOD_MS      10U    /* 100 Hz IMU sampling & filter update */
#define MPU_TELEMETRY_PERIOD_MS   20U    /* 50 Hz MPU-6050 & Pose VCP stream */
#define BMP_TELEMETRY_PERIOD_MS   100U   /* 10 Hz BMP280 VCP stream & reconnect */
#define LED_HEARTBEAT_PERIOD_MS   500U   /* 2 Hz LED status update */
#define UART_TX_BUF_SIZE          2048U  /* Power-of-2 non-blocking TX ring */
#define UART_TX_BUF_MASK          (UART_TX_BUF_SIZE - 1U)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart3;

static bmp280_t bmp280;
static mpu6050Data_t imuData;
static pose_t aircraftPose;

static uint8_t uartTxBuf[UART_TX_BUF_SIZE];
static volatile uint16_t uartTxHead = 0U;
static volatile uint16_t uartTxTail = 0U;
static volatile uint16_t uartTxActiveLen = 0U;
static volatile bool uartTxBusy = false;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static bmp280_status_t Probe_BMP280_I2C(uint8_t *out_addr);
static void Uart_StartNextTx(void);

static void Uart_StartNextTx(void)
{
  if (uartTxBusy || (uartTxHead == uartTxTail))
  {
    return;
  }

  uint16_t head = uartTxHead;
  uint16_t tail = uartTxTail;
  uint16_t len = (head > tail) ? (uint16_t)(head - tail) : (uint16_t)(UART_TX_BUF_SIZE - tail);

  uartTxActiveLen = len;
  uartTxBusy = true;

  if (HAL_UART_Transmit_IT(&huart3, &uartTxBuf[tail], len) != HAL_OK)
  {
    uartTxBusy = false;
    uartTxActiveLen = 0U;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uartTxTail = (uint16_t)((uartTxTail + uartTxActiveLen) & UART_TX_BUF_MASK);
    uartTxActiveLen = 0U;
    uartTxBusy = false;
    Uart_StartNextTx();
  }
}

int __io_putchar(int ch)
{
  uint8_t byte = (uint8_t)ch;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint16_t nextHead = (uint16_t)((uartTxHead + 1U) & UART_TX_BUF_MASK);
  if (nextHead != uartTxTail)
  {
    uartTxBuf[uartTxHead] = byte;
    uartTxHead = nextHead;
    Uart_StartNextTx();
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
  return ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  if ((ptr == NULL) || (len <= 0))
  {
    return 0;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint16_t used = (uint16_t)((uartTxHead - uartTxTail) & UART_TX_BUF_MASK);
  uint16_t freeSpace = (uint16_t)((UART_TX_BUF_SIZE - 1U) - used);

  /* Drop entire frame only if TX buffer overflows to avoid split lines */
  if ((uint16_t)len <= freeSpace)
  {
    for (int i = 0; i < len; i++)
    {
      uartTxBuf[uartTxHead] = (uint8_t)ptr[i];
      uartTxHead = (uint16_t)((uartTxHead + 1U) & UART_TX_BUF_MASK);
    }
    Uart_StartNextTx();
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
  return len;
}

static bmp280_status_t Probe_BMP280_I2C(uint8_t *out_addr)
{
  bmp280_status_t status = bmp280_init_default(&bmp280, &hi2c1, BMP280_I2C_ADDR_PRIM);
  if (status == BMP280_OK)
  {
    if (out_addr != NULL)
    {
      *out_addr = BMP280_I2C_ADDR_PRIM;
    }
    return BMP280_OK;
  }

  status = bmp280_init_default(&bmp280, &hi2c1, BMP280_I2C_ADDR_SEC);
  if ((status == BMP280_OK) && (out_addr != NULL))
  {
    *out_addr = BMP280_I2C_ADDR_SEC;
  }
  return status;
}

int main(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  setvbuf(stdout, NULL, _IONBF, 0);
  MX_I2C1_Init();

  printf("\r\n=======================================================================\r\n");
  printf("  NUCLEO-F767ZI Dual-Sensor Monitor: BMP280 (I2C) + MPU-6050 (I2C)\r\n");
  printf("  SYSCLK: 216MHz | I2C1: PB8(SCL)/PB9(SDA) @ 400kHz | VCP: PD8/PD9\r\n");
  printf("=======================================================================\r\n");

  /* Initialize BMP280 over I2C (0x76 if SDO=GND, 0x77 if SDO=3.3V, CSB=3.3V) */
  uint8_t bmpAddr = BMP280_I2C_ADDR_PRIM;
  printf("[INFO] Probing BMP280 over I2C (0x76 / 0x77)...\r\n");
  bmp280_status_t bmpStatus = Probe_BMP280_I2C(&bmpAddr);

  if (bmpStatus == BMP280_OK)
  {
    printf("[OK] BMP280 detected over I2C at 0x%02X! Chip ID: 0x58\r\n", bmpAddr);
    printf("[INFO] Calibration: dig_T1=%u, dig_T2=%d, dig_T3=%d\r\n",
           bmp280.calib.dig_T1, bmp280.calib.dig_T2, bmp280.calib.dig_T3);
    printf("[INFO] Calibration: dig_P1=%u, dig_P2=%d, dig_P3=%d\r\n",
           bmp280.calib.dig_P1, bmp280.calib.dig_P2, bmp280.calib.dig_P3);
  }
  else
  {
    printf("[ERROR] Failed to detect BMP280 over I2C (status: %d)!\r\n", bmpStatus);
    printf("        Check wiring: SCL=PB8(D15), SDA=PB9(D14), CSB=3.3V, SDO=GND(0x76)/3.3V(0x77)\r\n");
  }

  /* Initialize MPU-6050 over shared I2C1 bus (0x68 when AD0=GND) */
  printf("[INFO] Probing MPU-6050 over I2C at 0x%02X...\r\n", dMPU6050_I2C_ADDR_7BIT);
  bool mpuOk = Mpu6050_Init(&hi2c1);

  if (mpuOk)
  {
    printf("[OK] MPU-6050 detected over I2C at 0x%02X! WHO_AM_I: 0x%02X\r\n",
           dMPU6050_I2C_ADDR_7BIT, dMPU6050_WHO_AM_I_VALUE);
  }
  else
  {
    printf("[ERROR] Failed to detect MPU-6050 over I2C (0x%02X)!\r\n", dMPU6050_I2C_ADDR_7BIT);
    printf("        Check wiring: SCL=PB8(D15), SDA=PB9(D14), AD0=GND, VCC=3.3V\r\n");
  }

  ComplementaryFilter_Init(&aircraftPose, dFILTER_DEFAULT_ALPHA);

  if ((bmpStatus == BMP280_OK) && mpuOk)
  {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  }

  uint32_t lastImuTick = HAL_GetTick();
  uint32_t lastMpuTxTick = lastImuTick;
  uint32_t lastBmpTxTick = lastImuTick;
  uint32_t lastLedTick = lastImuTick;

  while (1)
  {
    uint32_t loopStart = HAL_GetTick();

    /* 1. 100 Hz MPU-6050 sampling + Complementary Filter update */
    if (mpuOk)
    {
      uint32_t dtMs = loopStart - lastImuTick;
      if (dtMs == 0U)
      {
        dtMs = IMU_SAMPLE_PERIOD_MS;
      }
      lastImuTick = loopStart;

      if (Mpu6050_ReadSensor(&imuData))
      {
        ComplementaryFilter_Update(&aircraftPose, &imuData, (float)dtMs / 1000.0f);
      }
      else
      {
        printf("[WARN] MPU-6050 read failed!\r\n");
        mpuOk = false;
      }
    }

    /* 2. 50 Hz MPU-6050 & Pose non-blocking serial stream */
    if ((loopStart - lastMpuTxTick) >= MPU_TELEMETRY_PERIOD_MS)
    {
      lastMpuTxTick = loopStart;
      if (mpuOk)
      {
        printf("[NUCLEO-F767ZI MPU6050 I2C] Acc: %+6.2f,%+6.2f,%+6.2f g | Gyro: %+7.2f,%+7.2f,%+7.2f dps | Temp: %5.2f degC | Pose R:%+6.1f P:%+6.1f Y:%+6.1f deg\r\n",
               imuData.accelXG, imuData.accelYG, imuData.accelZG,
               imuData.gyroXDps, imuData.gyroYDps, imuData.gyroZDps,
               imuData.tempC,
               aircraftPose.rollDeg, aircraftPose.pitchDeg, aircraftPose.yawDeg);
      }
    }

    /* 3. 10 Hz BMP280 read, sensor reconnect retry, and serial stream */
    if ((loopStart - lastBmpTxTick) >= BMP_TELEMETRY_PERIOD_MS)
    {
      lastBmpTxTick = loopStart;

      if (bmpStatus != BMP280_OK)
      {
        bmpStatus = Probe_BMP280_I2C(&bmpAddr);
        if (bmpStatus == BMP280_OK)
        {
          printf("[OK] BMP280 I2C reconnected at 0x%02X!\r\n", bmpAddr);
        }
      }

      if (!mpuOk)
      {
        mpuOk = Mpu6050_Init(&hi2c1);
        if (mpuOk)
        {
          ComplementaryFilter_Init(&aircraftPose, dFILTER_DEFAULT_ALPHA);
          lastImuTick = HAL_GetTick();
          printf("[OK] MPU-6050 I2C reconnected at 0x%02X!\r\n", dMPU6050_I2C_ADDR_7BIT);
        }
      }

      if (bmpStatus == BMP280_OK)
      {
        float temp_c = 0.0f;
        float press_pa = 0.0f;
        float alt_m = 0.0f;

        if (bmp280_read_data(&bmp280, &temp_c, &press_pa, &alt_m) == BMP280_OK)
        {
          printf("[NUCLEO-F767ZI BMP280 I2C] Temp: %6.2f degC | Press: %8.2f hPa (%7.0f Pa) | Hypsometric Alt: %7.2f m\r\n",
                 temp_c, press_pa / 100.0f, press_pa, alt_m);
        }
        else
        {
          printf("[WARN] BMP280 I2C read failed!\r\n");
          bmpStatus = BMP280_ERR_COMM;
        }
      }
    }

    /* 4. Visual heartbeat LED status */
    if ((loopStart - lastLedTick) >= LED_HEARTBEAT_PERIOD_MS)
    {
      lastLedTick = loopStart;
      if ((bmpStatus == BMP280_OK) && mpuOk)
      {
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      }
    }

    uint32_t elapsed = HAL_GetTick() - loopStart;
    if (elapsed < IMU_SAMPLE_PERIOD_MS)
    {
      HAL_Delay(IMU_SAMPLE_PERIOD_MS - elapsed);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* 16 MHz HSI -> PLL @ 216 MHz (PLLM=16, PLLN=432, PLLP=2, PLLQ=9) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Required on STM32F767 to sustain 216 MHz core clock */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; /* 54 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; /* 108 MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  /* 400 kHz Fast Mode @ I2CCLK = PCLK1 = 54 MHz (PRESC=5, SCLDEL=3, SDADEL=2, SCLH=7, SCLL=13) */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x5032070DU;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(USART3_IRQn, 3U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD2_Pin | LD3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LD1_Pin | LD2_Pin | LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    for (volatile int i = 0; i < 500000; i++);
  }
}
