/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

#include "basic.h"		// original library
#include "u8g2.h"		// graphic display library
#include "u8x8_gpio_STM32F303.h"
#include "u8x8_byte_4wire_hw_spi.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
u8g2_t u8g2; // a structure which will contain all the data for one display

// RPM bar graph parameter definition
#define	rpmbar_x		0
#define	rpmbar_y		0
#define	rpmbar_width	128
#define	rpmbar_height	14
#define	rpm_min			0
#define	rpm_max			9000

// measurements display parameter definition
#define	N_meas			6
#define	meas_x			0
#define	meas_y			16
#define	meas_x_offset	2
#define	meas_width1		70
#define	meas_width2		56
#define	meas_height		12

// indicators parameter definition
#define	N_idct			2
#define	idct_x			72
#define	idct_y			53
#define	idct_width		27
#define	idct_height		9

//  Fuel Pump Voltage dosplay parameter definition
#define	FP_x			0
#define	FP_y			52
#define	FP_height		12
#define	FP_volt_width	45
#define	FP_duty_width	70

// logo parameter definition
#define logo_width 48
#define logo_height 48

const unsigned char logo_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x3F, 0x00,
  0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xE0, 0x1F, 0x00, 0x80, 0xFF, 0x07,
  0xE0, 0x01, 0x00, 0x80, 0x81, 0x07, 0xE0, 0x01, 0x00, 0x80, 0x81, 0x07,
  0xC0, 0xE1, 0x07, 0xC0, 0x80, 0x03, 0x00, 0xFC, 0x1F, 0xE0, 0x80, 0x03,
  0x00, 0xFE, 0x7F, 0x70, 0xC0, 0x01, 0x00, 0x0F, 0xFE, 0x39, 0xE0, 0x00,
  0x00, 0x07, 0xF8, 0x1F, 0xF0, 0x00, 0x00, 0x0E, 0xE0, 0x1F, 0x78, 0x00,
  0x00, 0x0C, 0xC0, 0xFF, 0x3F, 0x00, 0x00, 0x18, 0x00, 0xFF, 0x1F, 0x00,
  0x00, 0x38, 0xC0, 0xFF, 0x0F, 0x00, 0x00, 0x70, 0xE0, 0xF1, 0x03, 0x00,
  0x00, 0xE0, 0x78, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0xE0, 0x01, 0x00,
  0x00, 0x80, 0x1F, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x07, 0xF8, 0x00, 0x00,
  0x00, 0x00, 0x1E, 0x7C, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00,
  0x00, 0x00, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x1C, 0x00, 0x00, 0x00,
  0x00, 0x32, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0x00, 0x18, 0x00,
  0x00, 0x7C, 0xEF, 0xFC, 0x39, 0x00, 0x00, 0x4E, 0xC6, 0xB8, 0x31, 0x00,
  0x00, 0x3E, 0xC6, 0x98, 0x31, 0x00, 0x00, 0x3C, 0xC6, 0x98, 0x31, 0x00,
  0x00, 0x0E, 0xC6, 0x98, 0x31, 0x00, 0x00, 0x4E, 0xCE, 0x99, 0x31, 0x00,
  0x00, 0x7C, 0xEF, 0xFD, 0x7B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  };

/*
#define logo_width 54
#define logo_height 48

static unsigned char logo_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0x0F,
  0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0xE0, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0x01, 0xF8, 0x3F, 0x00, 0x00, 0xF0, 0xFF, 0x07, 0xF8,
  0x01, 0x00, 0x00, 0x70, 0xE0, 0x0F, 0x7C, 0x00, 0x00, 0x00, 0x70, 0x80,
  0x0F, 0x7C, 0x00, 0x00, 0x00, 0x70, 0x00, 0x07, 0xF8, 0x00, 0x00, 0x00,
  0x38, 0x80, 0x07, 0x00, 0xF8, 0x3F, 0x00, 0x38, 0x80, 0x07, 0x00, 0xFE,
  0xFF, 0x00, 0x1C, 0xC0, 0x03, 0x80, 0xFF, 0xFF, 0x01, 0x0E, 0xC0, 0x01,
  0xC0, 0x07, 0xFC, 0x07, 0x07, 0xE0, 0x01, 0x80, 0x03, 0xF0, 0x9F, 0x03,
  0xF0, 0x00, 0x80, 0x03, 0xC0, 0xFF, 0x03, 0x78, 0x00, 0x00, 0x07, 0x80,
  0xFF, 0x01, 0x3C, 0x00, 0x00, 0x06, 0x00, 0xFE, 0x87, 0x3F, 0x00, 0x00,
  0x0E, 0x00, 0xF8, 0xFF, 0x1F, 0x00, 0x00, 0x1C, 0x00, 0xF8, 0xFF, 0x07,
  0x00, 0x00, 0x38, 0x00, 0xDE, 0xFF, 0x03, 0x00, 0x00, 0x70, 0x00, 0x0F,
  0x7E, 0x00, 0x00, 0x00, 0xE0, 0xC0, 0x07, 0x00, 0x00, 0x00, 0x00, 0xC0,
  0xF1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x00, 0x7C, 0x00, 0x00,
  0x00, 0x00, 0x3F, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x1F,
  0x00, 0x00, 0x00, 0x00, 0x3C, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x78,
  0xE0, 0x07, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0x03, 0x00, 0x00, 0x00,
  0x00, 0xC0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x4E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39,
  0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7E, 0x3E, 0xC7, 0x3D, 0x1C, 0x00, 0x00, 0x6F, 0x3E, 0xC7, 0x7F,
  0x1C, 0x00, 0x00, 0x03, 0x1C, 0x86, 0x73, 0x18, 0x00, 0x00, 0x07, 0x1C,
  0x86, 0x71, 0x18, 0x00, 0x00, 0x3E, 0x1C, 0x86, 0x71, 0x18, 0x00, 0x00,
  0x1F, 0x1C, 0x86, 0x71, 0x18, 0x00, 0x00, 0x03, 0x1C, 0x86, 0x71, 0x18,
  0x00, 0x00, 0x47, 0x1C, 0x86, 0x71, 0x18, 0x00, 0x00, 0x7F, 0x3E, 0xCF,
  0xFB, 0x3C, 0x00, 0x00, 0x3C, 0x3E, 0xCF, 0xFB, 0x3C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  };
*/

uint16_t	rpm = 0;
uint16_t	speed = 240;
uint8_t		gear = 0;
double		MT[5] = {3.483, 2.015, 1.391, 1.000, 0.806};

/*
typedef struct {
	unsigned char	name;
	uint8_t			status;
} idct;
*/

// variables for indicators
const unsigned char idct_name[N_idct][5] = { // length must be (text length + 1)
		"CAN"	,
		"O2FB"
};

const uint8_t	idct_status[N_idct] = {
		1	,
		0
};


// variables for measurement
const unsigned char meas_name[N_meas][7] = { // length must be (text length + 1)
		"MAP"	,
		"OILP"	,
		"FPVOLT"	,
		"ECT"	,
		"OILT"	,
		"O2"
};

const unsigned char meas_unit[N_meas][5] = {
		"kpa"			,	// MAP
		"kpa"			,	// OILP
		"V  "     ,	// FuelPump Voltage
		{176, 67, 0}	,	// ECT  degC...{176, 67, 0}
		{176, 67, 0}	,	// OILT  degC...{176, 67, 0}
		"V"					// O2
};

const uint8_t	meas_digit[N_meas] = {
		3	,	// MAP
		3	,	// OILP
		3	,	// FuelPump Voltage
		3	,	// ECT
		3	,	// OILT
		3		// O2
};

const uint8_t	meas_frac[N_meas] = {
		2	,	// MAP
		2	,	// OILP
		1	,	// FuelPump Voltage
		0	,	// ECT
		0	,	// OILT
		2		// O2
};

const uint8_t	meas_sign[N_meas] = {
		1	,	// MAP
		0	,	// OILP
		0	,	// FuelPump Voltage
		0	,	// ECT
		0	,	// OILT
		0		// O2
};

int16_t	meas_value[N_meas] = {
		645	,	// FuelPump Voltage
		105	,	// MAP
		235	,	// OILP
		89	,	// ECT
		73	,	// OILT
		143		// O2
};

// Fuel Pump Voltage
int16_t FP_volt = 330;
int16_t FP_duty = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint8_t n;
  uint8_t x, y;

  uint16_t a = 0;
  uint16_t b = 0;


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */

  // ADC buffer definition
  enum{ ADC_BUFFER_LENGTH = 1024 };
  uint16_t g_ADCBuffer[ADC_BUFFER_LENGTH];
  memset(g_ADCBuffer, 0, sizeof(g_ADCBuffer));
  HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, ADC_BUFFER_LENGTH);

  // OLED diaplay initialization
  u8g2_Setup_ssd1309_128x64_noname2_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_gpio_and_delay_STM32F303);  // init u8g2 structure
  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_3, 1); // Set RES=H (OLED activate)
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  u8g2_SetContrast(&u8g2, 128); // set contrast
  u8g2_ClearDisplay(&u8g2);

  u8g2_ClearBuffer(&u8g2);


  // draw opening
  u8g2_DrawXBMP(&u8g2, 40, 0, logo_width, logo_height, logo_bits );

  u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
  u8g2_DrawStr(&u8g2, 16, 63 - 8, "Multi Function Meter");
  u8g2_DrawStr(&u8g2, 40, 64, "Rev. 0.1a");

  u8g2_SendBuffer(&u8g2);

  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  u8g2_ClearBuffer(&u8g2);


  // draw measurements label & unit
  for( n=0; n<3; n++ ){
	  x = meas_x;
	  y = (n % 3) * meas_height	+ meas_y;
	  draw_MeasLabelUnit(&u8g2, x, y, meas_width1, meas_height, meas_name[n], meas_unit[n]);
  }
  for( n=3; n<N_meas; n++ ){
	  x = meas_width1 + meas_x + meas_x_offset;
	  y = (n % 3) * meas_height	+ meas_y;
	  draw_MeasLabelUnit(&u8g2, x, y, meas_width2, meas_height, meas_name[n], meas_unit[n]);
  }


  // draw indicators
  for( n=0; n<N_idct; n++ ){
	  draw_IndicatorBox(&u8g2, idct_x+(idct_width+2)*n, idct_y, idct_width, idct_height, idct_status[n], idct_name[n]);
  }
  u8g2_SendBuffer(&u8g2);

  // Fuel Pump voltage & duty
  
  draw_MeasLabelUnit(&u8g2, FP_x, FP_y, FP_duty_width, FP_height, "DUTY", "%  ");
  draw_Value(&u8g2, FP_x, FP_y, FP_duty_width, FP_height, 100, 3, 0, 0, "%  ");

  // update display
  u8g2_SendBuffer(&u8g2);


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  	// start of create dummy data for debug
		if( rpm <= 8200 ){
			  rpm = rpm + (int)(10*MT[gear]);
//			  rpm=rpm+10;
		}else{
			if( gear < 4 ){
				gear++;
				rpm = (double)speed / (0.002*60.0) * (MT[gear]*4.1);
			}else{
				gear = 0;
				rpm = 1000;
			}
		}
		speed = (double)rpm*0.002*60.0/MT[gear]/4.1;
		meas_value[0] = speed; // speed

		if( a >= 150){
			a = 0;
		}else{
			a++;
		}
		for( n=1; n<=5; n++){
			if( n==1 ){
				meas_value[n] = a - 75;
			}else{
				meas_value[n] = a;
			}
		}

	  	// end of create dummy data for debug


		// read O2 sensor ADC output
		meas_value[5] = (int16_t)(330 * (float)g_ADCBuffer[0]/255);

		// read Fuel Pump Voltage ADC output
		FP_volt = (int16_t)(330*(float)g_ADCBuffer[0]/255);
    FP_duty = (int16_t)(FP_volt/(14.4)*10);
		meas_value[2] = FP_volt;

		// update Fuel Pump Voltage
    draw_Value(&u8g2, FP_x, FP_y, FP_duty_width, FP_height, FP_duty, 3, 0, 0, "%  ");

		// draw bar graph
		draw_BarGraph(&u8g2, rpmbar_x, rpmbar_y, rpmbar_width, rpmbar_height, rpm, rpm_min, rpm_max);

		// draw measurement data
		for( n=0; n<3; n++ ){
			x = meas_x;
			y = (n % 3) * meas_height	+ meas_y;
			draw_Value(&u8g2, x, y, meas_width1, meas_height, meas_value[n], meas_digit[n], meas_frac[n], meas_sign[n], meas_unit[n]);
		}
		for( n=3; n<N_meas; n++ ){
			x = meas_width1 + meas_x + meas_x_offset;
			y = (n % 3) * meas_height	+ meas_y;
			draw_Value(&u8g2, x, y, meas_width2, meas_height, meas_value[n], meas_digit[n], meas_frac[n], meas_sign[n], meas_unit[n]);
		}


		// send buffer
	    u8g2_SendBuffer(&u8g2);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d¥r¥n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
