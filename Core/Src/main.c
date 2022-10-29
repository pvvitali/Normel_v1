/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "Lcd_1602.h"
#include "sim800.h"

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
 ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t time = 0;
volatile uint32_t time_init_uart = 0;
volatile uint32_t time_send_sim800 = 0;
volatile uint32_t time_adc_u = 0;

volatile uint8_t flag_adc_ready = 0;
volatile uint8_t flag_data_ready = 0;
volatile uint16_t adc[10] = {0,};
volatile uint16_t data_adc[10] = {0,};

int32_t value = 0;
int16_t mid_value_u1 = 2010;
int16_t mid_value_u2 = 2030;
int16_t mid_value_u3 = 2088;
int16_t mid_value_u4 = 1999;
int16_t mid_value_u5 = 2000;
int16_t mid_value_u6 = 2082;
int16_t mid_value_i1 = 2056;
int16_t mid_value_i2 = 2056;
int16_t mid_value_i3 = 2056;
//
int32_t amount_u1 = 0;
int32_t amount_u2 = 0;
int32_t amount_u3 = 0;
int32_t amount_u4 = 0;
int32_t amount_u5 = 0;
int32_t amount_u6 = 0;
int32_t amount_i1 = 0;
int32_t amount_i2 = 0;
int32_t amount_i3 = 0;
//
int32_t amount_u1_temp = 0;
int32_t amount_u2_temp = 0;
int32_t amount_u3_temp = 0;
int32_t amount_u4_temp = 0;
int32_t amount_u5_temp = 0;
int32_t amount_u6_temp = 0;
int32_t amount_i1_temp = 0;
int32_t amount_i2_temp = 0;
int32_t amount_i3_temp = 0;
//
uint16_t counter_sample = 0;
uint16_t counter_sample_temp = 0;

float u1 = 0;
float u2 = 0;
float u3 = 0;
float u4 = 0;
float u5 = 0;
float u6 = 0;
float i1 = 0;
float i2 = 0;
float i3 = 0;
//
float u1_average = 0;
float u2_average = 0;
float u3_average = 0;
float u4_average = 0;
float u5_average = 0;
float u6_average = 0;
float i1_average = 0;
float i2_average = 0;
float i3_average = 0;
uint16_t counter_average = 0;
//
float u1_average_gsm = 0;
float u2_average_gsm = 0;
float u3_average_gsm = 0;
float u4_average_gsm = 0;
float u5_average_gsm = 0;
float u6_average_gsm = 0;
float i1_average_gsm = 0;
float i2_average_gsm = 0;
float i3_average_gsm = 0;
uint16_t counter_average_gsm = 0;



//UART
uint8_t uart_count = 0;
uint8_t data_uart_receive = 0;
uint8_t data_mas_uart[100] = {0,};
uint8_t flag_uart_init = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	if(hadc->Instance == ADC1){
		flag_adc_ready = 1;
	}
}


//Timer 6
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
        if(htim->Instance == TIM6){ //check if the interrupt comes from TIM6
									
						//copy adc to adc temp bufer, adc set zero;
						for( uint8_t i = 0; i<10; i++){
							data_adc[i] = adc[i];
							adc[i] = 0;
						}
						
						HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc, 9);
						
						flag_adc_ready = 0;
						flag_data_ready = 1;
        }
}



//UART1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

		data_mas_uart[uart_count] = data_uart_receive; 
		uart_count++;
		if( uart_count > 99 ) uart_count = 0;
	
		HAL_UART_Receive_IT( huart, (uint8_t*) &data_uart_receive, 1);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_USART1_UART_Init();
//  MX_USART2_UART_Init();
//  MX_TIM6_Init();
//  MX_ADC_Init();
//  MX_I2C2_Init();

	
	
	//------------------------------------------
	HAL_ADCEx_Calibration_Start(&hadc);
	//HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc, 9);
	//HAL_ADC_Start_IT(&hadc);  
	
	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF); // очищаем флаг
	HAL_TIM_Base_Start_IT(&htim6);
	
	HAL_UART_Receive_IT( &huart1, (uint8_t*) &data_uart_receive, 1);
	
	HAL_Delay(100);
	
  Init_lcd_1602(&hi2c2);
  lcd_led_on();
  char mas_char[25];
  int count=0;
	

	//------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  {
		
		if( flag_data_ready ){
			
			
				value = data_adc[0];
				value -= mid_value_u1;
				amount_u1 += value*value;
			
				value = data_adc[1];
				value -= mid_value_u2;
				amount_u2 += value*value;
			
				value = data_adc[2];
				value -= mid_value_u3;
				amount_u3 += value*value;
			
				value = data_adc[3];
				value -= mid_value_u4;
				amount_u4 += value*value;
			
				value = data_adc[4];
				value -= mid_value_u5;
				amount_u5 += value*value;
			
				value = data_adc[5];
				value -= mid_value_u6;
				amount_u6 += value*value;
			
				value = data_adc[6];
				value -= mid_value_i1;
				amount_i1 += value*value;
			
				value = data_adc[7];
				value -= mid_value_i2;
				amount_i2 += value*value;
			
				value = data_adc[8];
				value -= mid_value_i3;
				amount_i3 += value*value;
			
				counter_sample++;
				
				//zero detector
				if( counter_sample > 199 ){
					
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					
					amount_u1_temp = amount_u1;
					amount_u2_temp = amount_u2;
					amount_u3_temp = amount_u3;
					amount_u4_temp = amount_u4;
					amount_u5_temp = amount_u5;
					amount_u6_temp = amount_u6;
					amount_i1_temp = amount_i1;
					amount_i2_temp = amount_i2;
					amount_i3_temp = amount_i3;
					counter_sample_temp = counter_sample;
					
					amount_u1 = 0;
					amount_u2 = 0;
					amount_u3 = 0;
					amount_u4 = 0;
					amount_u5 = 0;
					amount_u6 = 0;
					amount_i1 = 0;
					amount_i2 = 0;
					amount_i3 = 0;
					counter_sample = 0;
					
															
					u1 = sqrt((float)amount_u1_temp/counter_sample_temp)/3.30;
					u2 = sqrt((float)amount_u2_temp/counter_sample_temp)/3.32;
					u3 = sqrt((float)amount_u3_temp/counter_sample_temp)/3.32;
					u4 = sqrt((float)amount_u4_temp/counter_sample_temp)/3.32;
					u5 = sqrt((float)amount_u5_temp/counter_sample_temp)/3.32;
					u6 = sqrt((float)amount_u6_temp/counter_sample_temp)/3.31;
					i1 = sqrt((float)amount_i1_temp/counter_sample_temp)/64.94;
					i2 = sqrt((float)amount_i2_temp/counter_sample_temp)/64.94;
					i3 = sqrt((float)amount_i3_temp/counter_sample_temp)/64.94;
					
					u1_average += u1;
					u2_average += u2;
					u3_average += u3;
					u4_average += u4;
					u5_average += u5;
					u6_average += u6;
					i1_average += i1;
					i2_average += i2;
					i3_average += i3;					
					counter_average++;

				}
			

			flag_data_ready = 0;
		}			
		
		
		
		
		
		
		if( ( HAL_GetTick() - time ) > 1000 ){ // intetval 1000ms = 1s
			
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			
					u1_average /= counter_average;
					u2_average /= counter_average;
					u3_average /= counter_average;
					u4_average /= counter_average;
					u5_average /= counter_average;
					u6_average /= counter_average;
					i1_average /= counter_average;
					i2_average /= counter_average;
					i3_average /= counter_average;
	
					
					u1_average_gsm += u1_average;
					u2_average_gsm += u2_average;
					u3_average_gsm += u3_average;
					u4_average_gsm += u4_average;
					u5_average_gsm += u5_average;
					u6_average_gsm += u6_average;
					i1_average_gsm += i1_average;
					i2_average_gsm += i2_average;
					i3_average_gsm += i3_average;					
					counter_average_gsm++;			
					
			
					//print
					sprintf(mas_char,"%3u   %3u   %3u", (int)(u1_average + 0.5), (int)(u2_average + 0.5), (int)(u3_average + 0.5) );
					Lcd_1602_SetPos(&hi2c2, 0, 0);
					Lcd_1602_Write_Data(&hi2c2, (uint8_t *)mas_char);
					//
					//
					sprintf(mas_char,"%3u   %3u   %3u", (int)(u4_average + 0.5), (int)(u5_average + 0.5), (int)(u6_average + 0.5) );
					Lcd_1602_SetPos(&hi2c2, 0, 1);
					Lcd_1602_Write_Data(&hi2c2, (uint8_t *)mas_char);
					//
					//
					sprintf(mas_char,"%4.1f %4.1f %4.1f", i1_average, i2_average, i3_average);
					Lcd_1602_SetPos(&hi2c2, 0, 2);
					Lcd_1602_Write_Data(&hi2c2, (uint8_t *)mas_char);
					
					u1_average = 0;
					u2_average = 0;
					u3_average = 0;
					u4_average = 0;
					u5_average = 0;
					u6_average = 0;
					i1_average = 0;
					i2_average = 0;
					i3_average = 0;		
					counter_average = 0;
					
			
					time = HAL_GetTick();
		}
		
		

		
		
		if( ( HAL_GetTick() - time_init_uart ) > 40000 ){ // intetval 1000ms = 1s
			
				if( flag_uart_init == 0 ){
						
						init_sim800_udp(&huart1);
					
						flag_uart_init = 1;		//do not call init next time
				}
			
				time_init_uart = HAL_GetTick();
		}
		
		
		
		if( ( HAL_GetTick() - time_send_sim800 ) > 30000 ){ // intetval 1000ms = 1s
			
				if( flag_uart_init == 1 ){
					
						u1_average_gsm /= counter_average_gsm;
						u2_average_gsm /= counter_average_gsm;
						u3_average_gsm /= counter_average_gsm;
						u4_average_gsm /= counter_average_gsm;
						u5_average_gsm /= counter_average_gsm;
						u6_average_gsm /= counter_average_gsm;
						i1_average_gsm /= counter_average_gsm;
						i2_average_gsm /= counter_average_gsm;
						i3_average_gsm /= counter_average_gsm;
										
						sim800_send_udp_data( &huart1, (int)(u1_average_gsm + 0.5), (int)(u2_average_gsm + 0.5), (int)(u3_average_gsm + 0.5), (int)(u4_average_gsm + 0.5), (int)(u5_average_gsm + 0.5), (int)(u6_average_gsm + 0.5), i1_average_gsm, i2_average_gsm, i3_average_gsm);
																
						u1_average_gsm = 0;
						u2_average_gsm = 0;
						u3_average_gsm = 0;
						u4_average_gsm = 0;
						u5_average_gsm = 0;
						u6_average_gsm = 0;
						i1_average_gsm = 0;
						i2_average_gsm = 0;
						i3_average_gsm = 0;		
						counter_average_gsm = 0;
				}
				
				time_send_sim800 = HAL_GetTick();
		}
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303EFD;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
