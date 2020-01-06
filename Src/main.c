/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

// #include "basic.h"  // original library
#include "usart_transmit_printf.h"
#include "u8g2.h"		// graphic display library
#include "u8x8_gpio_STM32F303.h"
#include "u8x8_byte_4wire_hw_spi.h"
#include "draw_Value.h"
#include "draw_BarGraph.h"
#include "draw_IndicatorBox.h"
//#include "draw_Rotary.h"

#include "flag.h"
#include "defi_decoder.h"
#include "pulse_counter.h"
#include "can_obd.h"
#include "gsens_ADXL345.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// mode definition
#define N_mode                5

#define MODE_BAR_METER        0
#define MODE_CIRCULAR_METER   1
//#define MODE_ROTARY           2
#define MODE_SCOPE_MAP        2
#define MODE_SCOPE_Gsens      3
#define MODE_SETTING          4

// RPM bar graph parameter definition
#define	rpmbar_x		  0
#define	rpmbar_y		  0
#define	rpmbar_width	128
#define	rpmbar_height	14
#define	rpm_min			  0
#define	rpm_max			  9000

// measurements display parameter definition
// Bar graph
#define	N_meas			      6
#define	meas_x			      0
#define	meas_y			      16
#define	meas_x_offset   	2
#define	meas_width1		    70
#define	meas_width2		    56
#define	meas_height		    12
// Circular Meter
#define	N_meas_C			    4
#define	meas_C_x			    68
#define	meas_C_y			    2
#define	meas_C_x_offset		2
#define	meas_C_width		  60
#define	meas_C_height		  13

// indicators parameter definition
#define	N_idct			      2
#define	idct_x			      72
#define	idct_y			      53
#define	idct_width		    27
#define	idct_height		    9

//  Fuel Pump Voltage display parameter definition
#define	FP_x		    	    0
#define	FP_y              52
#define	FP_height	        12
#define	FP_volt_width	    45
#define	FP_duty_width	    70

// wave display parameter definition
#define	wave_x			      0
#define	wave_y			      13
#define	wave_width		    128
#define	wave_height		    52
#define	wave_value_min	  -100
#define	wave_value_max	  +200

// logo parameter definition (small 'enfini' logo)
#define logo_width        48
#define logo_height       48

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
// logo parameter definition (meidium 'enfini' logo)
#define logo_width        54
#define logo_height       48

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

u8g2_t    u8g2; // a structure which will contain all the data for one display

uint8_t   update_display = 0;

// uint16_t	rpm = 0;
uint16_t	speed = 240;
uint8_t		gear = 0;
double		MT[5] = {3.483, 2.015, 1.391, 1.000, 0.806};

// UI
uint8_t   mode;
uint8_t   setting;
uint8_t   cursor = 0;

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
		1
};

// variables for measurement
const unsigned char meas_name[N_meas][7] = { // length must be (text length + 1)
		"MAP"         ,
		"OILP"        ,
		"FPV"         ,
		"ECT"         ,
		"OILT"        ,
		"O2"          
};

const unsigned char meas_unit[N_meas][5] = {
		"kpa"			    ,	// MAP
		"kpa"			    ,	// OILP
		"V  "         ,	// FuelPump Voltage
		{176, 67, 0}  ,	// ECT  degC...{176, 67, 0}
		{176, 67, 0}	,	// OILT  degC...{176, 67, 0}
		"V"					    // O2
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
		1	,	// OILP
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
		105	,	// MAP
		235	,	// OILP
		645	,	// FuelPump Voltage
		89	,	// ECT
		73	,	// OILT
		143		// O2
};

// O2 senser Voltage
int16_t   O2_volt = 330;

// Fuel Pump Voltage
int16_t   FP_volt = 330;
int16_t   FP_duty = 100;

// ADXL345 3-axis acceration sensor --------------------------------
uint8_t   Gsens_EN;
uint8_t   Gsens_X1;
uint8_t   Gsens_X0;
int16_t   Gsens_X;
uint8_t   Gsens_Y1;
uint8_t   Gsens_Y0;
int16_t   Gsens_Y;
uint8_t   Gsens_Z1;
uint8_t   Gsens_Z0;
int16_t   Gsens_Z;

#define ADXL0_ADDR 0x1D
// ADXL345     I2C
// SDO/ALT   Address
//    H        0x1D
//    L        0x53

/*
void ADXL345_RegWrite(uint8_t slv_addr, uint8_t addr, uint8_t data){
  uint8_t i2c_buf[2];
  i2c_buf[0] = addr;
  i2c_buf[1] = data;  
  HAL_I2C_Master_Transmit(&hi2c1, slv_addr << 1, &i2c_buf, 2, 10);
}

int8_t ADXL345_RegRead_1byte(uint8_t slv_addr, uint8_t addr){
  uint8_t data;
  HAL_I2C_Master_Transmit(&hi2c1, slv_addr << 1, &addr, 1, 10);
  HAL_I2C_Master_Receive(&hi2c1, slv_addr << 1, &data, 1, 10);
  return data;
}
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void draw_indicators(){
  uint8_t n;
  uint8_t x, y;
  // draw indicators
  for( n=0; n<N_idct; n++ ){
	  draw_IndicatorBox(&u8g2, idct_x+(idct_width+2)*n, idct_y, idct_width, idct_height, idct_status[n], idct_name[n]);
  }
}

void draw_MeasLabels(){
  uint8_t n;
  uint8_t x, y;
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

  // Fuel Pump voltage & duty
  draw_MeasLabelUnit(&u8g2, FP_x, FP_y, FP_duty_width, FP_height, "DUTY", "%  ");
  draw_Value(&u8g2, FP_x, FP_y, FP_duty_width, FP_height, 100, 3, 0, 0, "%  ");
}

void draw_MeasLabels_Rotary(){
  uint8_t n;
  uint8_t x, y;
  // draw measurements label & unit
  for( n=0; n<4; n++ ){
	  x = meas_x;
	  y = (n % 4) * meas_height	+ meas_y;
	  draw_MeasLabelUnit(&u8g2, x, y, meas_width1, meas_height, meas_name[n], meas_unit[n]);
  }
}


// ADC buffer definition
enum{ ADC_BUFFER_LENGTH = 10 };
uint16_t g_ADCBuffer[ADC_BUFFER_LENGTH];
uint16_t adc[ADC_BUFFER_LENGTH];  

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	for (unsigned int i =0; i<ADC_BUFFER_LENGTH; i++){
	   adc[i] = g_ADCBuffer[i];  // store the values in adc[]
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uint8_t n;
  uint8_t x, y;

  uint8_t a = 0; // for dummy data
  uint8_t b = 0; // for dummy data

  // ROTARY
  uint16_t rpm_integral;
  uint8_t index_animation;

  // circular buffer for ADC data
  uint16_t circular_buffer_index = 0;
  int16_t circular_buffer[128]={};

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  ///// PWM initialize ----------------------------------------------------------------

  ///// Timer /////
  // TIM1 - PWM for Fuel Pump Driver (Cycle 100kHz = 1MHz / 100count)
  /*
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  */
  // Start Timer1 / ch1
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // TIM2 - display (Cycle 20ms = 1/100kHz x 2,000count)
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  // TIM3 - Pulse counter for Tacho (f 200kHz, count 65535)
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  // TIM6 - SW interrupt control (Cycle 100ms = 1/100kHz x 10,000count)
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  // ADC
  HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, ADC_BUFFER_LENGTH);

  // UART1 interrupt setup for DEFI decoder
  HAL_UART_Receive_IT(&huart1, &UART1_Data, 1);
  // variables is defined in 'defi.h'

  // CAN initialization
  CAN_OBD_Init();


  // OLED diaplay initialization
  u8g2_Setup_ssd1309_128x64_noname2_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_gpio_and_delay_STM32F303);  // init u8g2 structure

  // OLED display Reset (must be more than 3us!!)
  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_3, 0); // Set RES=L (OLED Reset)
  HAL_Delay(5);
  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_3, 1); // Set RES=H (OLED activate)
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  u8g2_SetContrast(&u8g2, 255); // set contrast
  u8g2_ClearDisplay(&u8g2);

  // draw opening
  u8g2_DrawXBMP(&u8g2, 40, 0, logo_width, logo_height, logo_bits );

  u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
  u8g2_DrawStr(&u8g2, 16, 63 - 8, "Multi Function Meter");
  u8g2_DrawStr(&u8g2, 40, 64, "Rev. 0.2a");
  u8g2_SendBuffer(&u8g2);

  HAL_Delay(1000);

  u8g2_ClearBuffer(&u8g2);

  HAL_UART_Transmit_printf(&huart2, "Hello. This is MFM.\n");

  // Set PWM Duty for Timer1 / Output1 (Asymmetric PWM2)
  TIM1->CCR1 = (100 - FP_duty);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  draw_MeasLabels();
  draw_indicators();
  u8g2_SendBuffer(&u8g2);


  // I2C communication to ADXL345(3-axis G-sensor)
  Gsens_EN = Gsens_ADXL345_Init(0);
/*
  // DEIVID
  a = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x00);
  HAL_UART_Transmit_printf(&huart2, "ADXL324 DEVID %d\n", a); // debug
  if( a == 0xE5 ){
    Gsens_EN = 1;
  }else{
    Gsens_EN = 0;
  }

  // POWER_CTL
  ADXL345_RegWrite(ADXL0_ADDR, 0x2D, 0x08);
  // bit 3    Measure   1'b1
  a = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x2D); // Check

  // DATA_FORMAT
  ADXL345_RegWrite(ADXL0_ADDR, 0x31, 0x05);
  // bit 2    Justify   1'b1
  // bit 1:0  Range     2'b01
  a = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x31); // Check "POWER_CTL"
*/

  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    ///// ADC ----------------------------------------------------------------

		// read O2 sensor ADC output
//		O2_volt = (int16_t)(330 * (float)g_ADCBuffer[0]/255);
		O2_volt = (int16_t)(330 * (float)adc[0]/255);

		// read Fuel Pump Voltage ADC output
//		FP_volt = (int16_t)((165/33)*33*(float)g_ADCBuffer[1]/255);
		FP_volt = (int16_t)((165/33)*33*(float)adc[1]/255);
    // Ressister attenation ratio '165/33'
    
    ///// Fuel Pump ----------------------------------------------------------------
    // Duty calculation
    if( (rpm >= 3000)|(DEFI_value[0] >= 0) ){
      FP_duty = 100;
    }else{
      FP_duty = (int16_t)(rpm/3000*100);
    }
    // Saturation process
    if( FP_duty > 100 ){
      FP_duty = 100;
    }else if( FP_duty < 60 ){
      FP_duty = 60;
    }

    ///// CAN ----------------------------------------------------------------

//    CAN_OBD_Response(MAP, rpm, SPEED, THROTTLE);
    CAN_OBD_Response(DEFI_value[0], rpm, 0x00, 0x00);

    ///// Measure data  ----------------------------------------------------------------
    defi_decoder(UART1_RxData); // DEFI decoder

    if( flag_meas == 1 ){

      // I2C communication to ADXL345(3-axis G-sensor)
      if( Gsens_EN == 1 ){
        /*
        // Justify = 0
        Gsens_X1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x33); // Check "DATAX1" (MSB side)
        Gsens_X0 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x32); // Check "DATAX0" (LSB side)
        Gsens_X = (Gsens_X1<<8) | Gsens_X0;
        Gsens_Y1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x35); // Check "DATAY1" (MSB side)
        Gsens_Y0 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x34); // Check "DATAY0" (LSB side)
        Gsens_Y = (Gsens_Y1<<8) | Gsens_Y0;
        Gsens_Z1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x37); // Check "DATAZ1" (MSB side)
        Gsens_Z0 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x36); // Check "DATAZ0" (LSB side)
        Gsens_Z = (Gsens_Z1<<8) | Gsens_Z0;
        HAL_UART_Transmit_printf(&huart2, "(%d,%d,%d)\n", Gsens_X, Gsens_Y, Gsens_Z); // debug
        */

        // Justify = 1
        Gsens_X1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x33); // Check "DATAX1" (MSB side)
        Gsens_Y1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x35); // Check "DATAY1" (MSB side)
        Gsens_Z1 = ADXL345_RegRead_1byte(ADXL0_ADDR, 0x37); // Check "DATAZ1" (MSB side)
        // Acceration 1G = 100
        Gsens_X = 400 * (int8_t)Gsens_X1 / 128; // unsigned->signed & scaling
        Gsens_Y = 400 * (int8_t)Gsens_Y1 / 128; // unsigned->signed & scaling
        Gsens_Z = 400 * (int8_t)Gsens_Z1 / 128; // unsigned->signed & scaling

        HAL_UART_Transmit_printf(&huart2, "(%d,%d,%d)\n", Gsens_X, Gsens_Y, Gsens_Z); // debug
        
        
      }


/*
  	  // start of create dummy data for debug
      if( rpm <= 8200 ){
          rpm = rpm + (int)(20*MT[gear]);
          //rpm=rpm+10;
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

      if( a >= 150){
        a = 0;
      }else{
        a++;
      }
  	  // end of create dummy data for debug
*/

      meas_value[0] = DEFI_value[0];  // MAP
		  meas_value[1] = DEFI_value[2];  // OILP
			meas_value[2] = FP_volt;        // FuelPump Voltage
		  meas_value[3] = DEFI_value[6];  // ECT
			meas_value[4] = DEFI_value[5];  // OILT
		  meas_value[5] = O2_volt;        // O2

      flag_meas = 0; // enable again by TIM2 interrupt
    }


    ///// FP driver ----------------------------------------------------------------
    // Set PWM Duty for Timer1 / Output1 (Asymmetric PWM2)
    TIM1->CCR1 = (100 - FP_duty);


    ///// Switch ----------------------------------------------------------------
    if( flag_sw != 0 ){
      switch( flag_sw ){
        case 1:
          HAL_UART_Transmit_printf(&huart2, "UP ");
          HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
          // SW "UP"
          if( setting == 0 ){
            if( mode == N_mode-1 ){
              mode = 0;
            }else{
              mode++;
            }
          }
          break;
        case 2:
          HAL_UART_Transmit_printf(&huart2, "DOWN ");
          HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
          // SW "DOWN"
          if( setting == 0 ){
            if( mode == 0 ){
              mode = N_mode-1;
            }else{
              mode--;
            }
          }
          break;
        case 3:
          HAL_UART_Transmit_printf(&huart2, "ENTER ");
          HAL_NVIC_DisableIRQ (EXTI1_IRQn);
          // SW "ENTER"

          // if( setting == 0 ){
          //   mode = cursor;
          //   u8g2_ClearBuffer(&u8g2);
          // }else{
          //   mode = 10;
          // }
          // break;

        default:
          break;
      }
      u8g2_ClearBuffer(&u8g2);
      if( mode == MODE_BAR_METER ){
        draw_MeasLabels();
        draw_indicators();

      }else if( mode == MODE_CIRCULAR_METER ){
        for( n=0; n<N_meas_C; n++ ){
          x = meas_C_x;
          y = n * meas_C_height	+ meas_C_y;
          draw_MeasLabelUnit(&u8g2, x, y, meas_C_width, meas_C_height, meas_name[n], meas_unit[n]);
        }
        // draw indicators
        for( n=0; n<4; n++ ){
          draw_IndicatorBox(&u8g2, idct_x+(idct_width+2)*n, idct_y, idct_width, idct_height, idct_status[n], idct_name[n]);
        }
        draw_CircularMeter_Init(32, 32, 31, 3, -60, 300, 10, 20, -1, -80, 120);
//        draw_CircularMeter_Init(26, 26, 26, 3, -60, 300, 10, 20, -1, -80, 120);
        draw_CircularMeter(&u8g2, 0);
/*
      }else if( mode == MODE_ROTARY ){
        draw_MeasLabels_Rotary();
        draw_MeasLabelUnit(&u8g2, 0, 2, meas_width1, meas_height, "TACHO", "rpm");
*/
      }else if( mode == MODE_SCOPE_MAP ){
        draw_Wave_axis(&u8g2, wave_x, wave_y, wave_width, wave_height, wave_value_min, wave_value_max, 3);
        draw_MeasLabelUnit(&u8g2, 0, 0, 64, 13, "MAP", "kPa");

      }else if( mode == MODE_SCOPE_Gsens ){
        draw_Wave_axis(&u8g2, wave_x, wave_y, wave_width, wave_height, -200, 200, 4);
        draw_MeasLabelUnit(&u8g2, 0, 0, 64, 13, "LatG", "G");

      }
      u8g2_SendBuffer(&u8g2);
      TIM6->CNT = 0;
      flag_sw = 0;
    }


    ///// Display sequence ----------------------------------------------------------------
    if( flag_disp ){

      ///// multi meter /////
      if( mode == MODE_BAR_METER ){
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
        
      ///// Circular Meter /////
      }else if( mode == MODE_CIRCULAR_METER ){

        draw_CircularMeter(&u8g2, meas_value[0]);
        
        draw_Value(&u8g2, 36, 33, 30, 16, meas_value[0], 3, 2, 1, "");
        draw_MeasUnit(&u8g2, 38, 40, 28, 16, "kPa");
/*        
        draw_Value(&u8g2, 34, 28, 30, 16, meas_value[0], 3, 2, 1, "");
        draw_MeasUnit(&u8g2, 36, 36, 28, 16, "kPa");
*/

        // draw measurement data
        for( n=0; n<N_meas_C; n++ ){
          x = meas_C_x;
          y = n * meas_C_height	+ meas_C_y;
          draw_Value(&u8g2, x, y, meas_C_width, meas_C_height, meas_value[n], meas_digit[n], meas_frac[n], meas_sign[n], meas_unit[n]);
        }

/*
      }else if( mode == MODE_ROTARY ){

        rpm_integral = rpm_integral + rpm;
        if( rpm_integral > 4000 ){
          rpm_integral = rpm_integral - 4000;
        }
        index_animation = (uint8_t)( (float)rpm_integral / 1000.0 );

        draw_Rotary(&u8g2, 58, 2, index_animation);

        draw_MeasLabelUnit(&u8g2, 0, 2, meas_width1, meas_height, "TACHO", "rpm");
        draw_Value(&u8g2, 0, 2, meas_width1, meas_height, rpm, 4, 0, 0, "rpm");

        draw_MeasLabels_Rotary();
        // draw measurement data
        for( n=0; n<4; n++ ){
          x = meas_x;
          y = (n % 4) * meas_height	+ meas_y;
          draw_Value(&u8g2, x, y, meas_width1, meas_height, meas_value[n], meas_digit[n], meas_frac[n], meas_sign[n], meas_unit[n]);
        }
*/

      ///// Scope /////
      }else if( mode == MODE_SCOPE_MAP ){

        if( circular_buffer_index > 0 ){
          circular_buffer_index--;
        }else{
          circular_buffer_index = 128;
        }
        circular_buffer[circular_buffer_index] = meas_value[0];

        // draw wave
        draw_Wave(&u8g2, wave_x, wave_y, wave_width, wave_height, wave_value_min, wave_value_max, circular_buffer, circular_buffer_index);
        draw_Value(&u8g2, 0, 0, 64, 13, circular_buffer[circular_buffer_index], 3, 2, 1, "kPa");

      ///// G-Scope /////
      }else if( mode == MODE_SCOPE_Gsens ){

        if( circular_buffer_index > 0 ){
          circular_buffer_index--;
        }else{
          circular_buffer_index = 128;
        }
        circular_buffer[circular_buffer_index] = Gsens_Y;

        // draw wave
        draw_Wave(&u8g2, wave_x, wave_y, wave_width, wave_height, -200, +200, circular_buffer, circular_buffer_index);
        draw_Value(&u8g2, 0, 0, 64, 13, circular_buffer[circular_buffer_index], 3, 2, 1, "G");

      // mode setting
      }else if( mode == MODE_SETTING ){

        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_7x14B_tf);
        u8g2_DrawStr(&u8g2, 2, 15, "Mode Setting" );
        if( cursor == 0 ){
          u8g2_DrawStr(&u8g2, 2, 30, ">" );
        }else{
          u8g2_DrawStr(&u8g2, 2, 30, " " );
        }
        u8g2_DrawStr(&u8g2,10, 30, "Multi-Meter" );
        if( cursor == 1 ){
          u8g2_DrawStr(&u8g2, 2, 45, ">" );
        }else{
          u8g2_DrawStr(&u8g2, 2, 45, " " );
        }
        u8g2_DrawStr(&u8g2,10, 45, "Scope" );
        u8g2_SendBuffer(&u8g2);
      }
      // send buffer
      u8g2_SendBuffer(&u8g2);

      flag_disp = 0; // enable again by TIM2 interrupt
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d¥r¥n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
