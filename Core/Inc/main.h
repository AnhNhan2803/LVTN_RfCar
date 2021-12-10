/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// Define GPIOs using for UART log debug
#define UART_TX_Pin                             GPIO_PIN_9
#define UART_TX_GPIO_Port                       GPIOA
#define UART_RX_Pin                             GPIO_PIN_10
#define UART_RX_GPIO_Port                       GPIOA

// Define GPIOs using for NRF24L01 communication
#define NRF24L01_SPI_SCK_Pin                    GPIO_PIN_5
#define NRF24L01_SPI_SCK_GPIO_Port              GPIOA 
#define NRF24L01_SPI_MISO_Pin                   GPIO_PIN_6
#define NRF24L01_SPI_MISO_GPIO_Port             GPIOA 
#define NRF24L01_SPI_MOSI_Pin                   GPIO_PIN_7
#define NRF24L01_SPI_MOSI_GPIO_Port             GPIOA 
#define NRF24L01_SPI_CS_Pin                     GPIO_PIN_8
#define NRF24L01_SPI_CS_GPIO_Port               GPIOA 
#define NRF24L01_CE_Pin                         GPIO_PIN_0
#define NRF24L01_CE_GPIO_Port                   GPIOB 
#define NRF24L01_IRQ_Pin                        GPIO_PIN_1
#define NRF24L01_IRQ_GPIO_Port                  GPIOB 

// Define GPIOs using for motor interface
#define MOTOR_LEFT1_Pin                         GPIO_PIN_0
#define MOTOR_LEFT1_GPIO_Port                   GPIOA
#define MOTOR_LEFT2_Pin                         GPIO_PIN_1
#define MOTOR_LEFT2_GPIO_Port                   GPIOA
#define MOTOR_RIGHT1_Pin                        GPIO_PIN_2
#define MOTOR_RIGHT1_GPIO_Port                  GPIOA
#define MOTOR_RIGHT2_Pin                        GPIO_PIN_3
#define MOTOR_RIGHT2_GPIO_Port                  GPIOA

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
