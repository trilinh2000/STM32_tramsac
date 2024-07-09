/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "Modbus.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern modbusHandler_t ModbusH;
extern modbusHandler_t ModbusH2;
extern uint16_t ModbusDATA[8];
extern uint16_t ModbusDATA2[8];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define X15_Pin GPIO_PIN_13
#define X15_GPIO_Port GPIOC
#define X00_Pin GPIO_PIN_0
#define X00_GPIO_Port GPIOA
#define X01_Pin GPIO_PIN_1
#define X01_GPIO_Port GPIOA
#define X12_Pin GPIO_PIN_2
#define X12_GPIO_Port GPIOA
#define X14_Pin GPIO_PIN_3
#define X14_GPIO_Port GPIOA
#define Y02_Pin GPIO_PIN_0
#define Y02_GPIO_Port GPIOB
#define Y03_Pin GPIO_PIN_1
#define Y03_GPIO_Port GPIOB
#define X13_Pin GPIO_PIN_2
#define X13_GPIO_Port GPIOB
#define X10_Pin GPIO_PIN_12
#define X10_GPIO_Port GPIOB
#define X11_Pin GPIO_PIN_13
#define X11_GPIO_Port GPIOB
#define X06_Pin GPIO_PIN_14
#define X06_GPIO_Port GPIOB
#define X07_Pin GPIO_PIN_15
#define X07_GPIO_Port GPIOB
#define X02_Pin GPIO_PIN_6
#define X02_GPIO_Port GPIOC
#define X03_Pin GPIO_PIN_7
#define X03_GPIO_Port GPIOC
#define Y05_Pin GPIO_PIN_9
#define Y05_GPIO_Port GPIOC
#define Y04_Pin GPIO_PIN_8
#define Y04_GPIO_Port GPIOA
#define X04_Pin GPIO_PIN_11
#define X04_GPIO_Port GPIOA
#define X05_Pin GPIO_PIN_12
#define X05_GPIO_Port GPIOA
#define Y00_Pin GPIO_PIN_10
#define Y00_GPIO_Port GPIOC
#define Y01_Pin GPIO_PIN_11
#define Y01_GPIO_Port GPIOC
#define Y07_Pin GPIO_PIN_3
#define Y07_GPIO_Port GPIOB
#define Y06_Pin GPIO_PIN_4
#define Y06_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
