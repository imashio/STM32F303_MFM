/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXTI__SW_Enter_Pin GPIO_PIN_1
#define EXTI__SW_Enter_GPIO_Port GPIOF
#define EXTI__SW_Enter_EXTI_IRQn EXTI1_IRQn
#define ADC__O2_Pin GPIO_PIN_0
#define ADC__O2_GPIO_Port GPIOA
#define ADC__FPVOLT_Pin GPIO_PIN_1
#define ADC__FPVOLT_GPIO_Port GPIOA
#define USART2__USB_TX_Pin GPIO_PIN_2
#define USART2__USB_TX_GPIO_Port GPIOA
#define EXTI__SW_Down_Pin GPIO_PIN_6
#define EXTI__SW_Down_GPIO_Port GPIOA
#define EXTI__SW_Down_EXTI_IRQn EXTI9_5_IRQn
#define OLED__CS_Pin GPIO_PIN_0
#define OLED__CS_GPIO_Port GPIOB
#define OLED__D_C_Pin GPIO_PIN_1
#define OLED__D_C_GPIO_Port GPIOB
#define USART1__Defi_TX_Pin GPIO_PIN_9
#define USART1__Defi_TX_GPIO_Port GPIOA
#define USART1__Defi_RX_Pin GPIO_PIN_10
#define USART1__Defi_RX_GPIO_Port GPIOA
#define USART2__USB_RX_Pin GPIO_PIN_15
#define USART2__USB_RX_GPIO_Port GPIOA
#define OLED__RES_Pin GPIO_PIN_3
#define OLED__RES_GPIO_Port GPIOB
#define EXTI__Tacho_Pin GPIO_PIN_4
#define EXTI__Tacho_GPIO_Port GPIOB
#define EXTI__Tacho_EXTI_IRQn EXTI4_IRQn
#define EXTI__SW_Up_Pin GPIO_PIN_5
#define EXTI__SW_Up_GPIO_Port GPIOB
#define EXTI__SW_Up_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
