/**
  ******************************************************************************
  * @file    main.h
  * @brief   Header for main.c module on NUCLEO-F767ZI (BMP280 + MPU-6050 I2C).
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"

void Error_Handler(void);

/* NUCLEO-F767ZI User Button & LEDs -----------------------------------------*/
#define USER_Btn_Pin            GPIO_PIN_13
#define USER_Btn_GPIO_Port      GPIOC

#define LD1_Pin                 GPIO_PIN_0
#define LD1_GPIO_Port           GPIOB
#define LD2_Pin                 GPIO_PIN_7
#define LD2_GPIO_Port           GPIOB
#define LD3_Pin                 GPIO_PIN_14
#define LD3_GPIO_Port           GPIOB

/* Shared I2C1 Bus on NUCLEO-144 Arduino Header (D15=PB8 SCL, D14=PB9 SDA) ---*/
#define SENSOR_I2C_SCL_Pin      GPIO_PIN_8
#define SENSOR_I2C_SCL_Port     GPIOB
#define SENSOR_I2C_SDA_Pin      GPIO_PIN_9
#define SENSOR_I2C_SDA_Port     GPIOB

/* ST-LINK Virtual COM Port (USART3 on PD8/PD9) ------------------------------*/
#define STLK_TX_Pin             GPIO_PIN_8
#define STLK_TX_GPIO_Port       GPIOD
#define STLK_RX_Pin             GPIO_PIN_9
#define STLK_RX_GPIO_Port       GPIOD

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
