/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define EXTI__SW_Enter_Pin GPIO_PIN_1
#define EXTI__SW_Enter_GPIO_Port GPIOF
#define EXTI__SW_Enter_EXTI_IRQn EXTI1_IRQn
#define ADC__O2_Pin GPIO_PIN_0
#define ADC__O2_GPIO_Port GPIOA
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

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
