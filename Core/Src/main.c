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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t time = 0;
volatile uint32_t time_init_uart = 0;
volatile uint32_t time_send_sim800 = 0;
volatile uint32_t time_adc_u = 0;

volatile uint8_t flag_adc_ready = 0;
volatile uint16_t adc[10] = {0,};
volatile uint16_t data_adc[10] = {0,};

uint16_t mid_value = 0;
////
//
uint16_t last_value_max_u1 = 0;
uint16_t last_value_min_u1 = 0;
uint16_t max_filtered_u1 = 0;
uint16_t min_filtered_u1 = 0;
uint32_t u1 = 0;
//
//
uint16_t last_value_max_u2 = 0;
uint16_t last_value_min_u2 = 0;
uint16_t max_filtered_u2 = 0;
uint16_t min_filtered_u2 = 0;
uint32_t u2 = 0;
//
//
uint16_t last_value_max_u3 = 0;
uint16_t last_value_min_u3 = 0;
uint16_t max_filtered_u3 = 0;
uint16_t min_filtered_u3 = 0;
uint32_t u3 = 0;
//
//
uint16_t last_value_max_u4 = 0;
uint16_t last_value_min_u4 = 0;
uint16_t max_filtered_u4 = 0;
uint16_t min_filtered_u4 = 0;
uint32_t u4 = 0;
//
//
uint16_t last_value_max_u5 = 0;
uint16_t last_value_min_u5 = 0;
uint16_t max_filtered_u5 = 0;
uint16_t min_filtered_u5 = 0;
uint32_t u5 = 0;
//
//
uint16_t last_value_max_u6 = 0;
uint16_t last_value_min_u6 = 0;
uint16_t max_filtered_u6 = 0;
uint16_t min_filtered_u6 = 0;
uint32_t u6 = 0;
//
//
uint16_t last_value_max_i1 = 0;
uint16_t last_value_min_i1 = 0;
uint16_t max_filtered_i1 = 0;
uint16_t min_filtered_i1 = 0;
uint32_t i1 = 0;
uint16_t i1_d = 0;
uint32_t i1_f = 0;
//
uint16_t last_value_max_i2 = 0;
uint16_t last_value_min_i2 = 0;
uint16_t max_filtered_i2 = 0;
uint16_t min_filtered_i2 = 0;
uint32_t i2 = 0;
uint16_t i2_d = 0;
uint32_t i2_f = 0;
//
uint16_t last_value_max_i3 = 0;
uint16_t last_value_min_i3 = 0;
uint16_t max_filtered_i3 = 0;
uint16_t min_filtered_i3 = 0;
uint32_t i3 = 0;
uint16_t i3_d = 0;
uint32_t i3_f = 0;
//
uint16_t last_value_max_i4 = 0;
uint16_t last_value_min_i4 = 0;
uint16_t max_filtered_i4 = 0;
uint16_t min_filtered_i4 = 0;
uint32_t i4 = 0;
uint16_t i4_d = 0;
uint32_t i4_f = 0;
//
uint16_t last_value_max_i5 = 0;
uint16_t last_value_min_i5 = 0;
uint16_t max_filtered_i5 = 0;
uint16_t min_filtered_i5 = 0;
uint32_t i5 = 0;
uint16_t i5_d = 0;
uint32_t i5_f = 0;
//
uint16_t last_value_max_i6 = 0;
uint16_t last_value_min_i6 = 0;
uint16_t max_filtered_i6 = 0;
uint16_t min_filtered_i6 = 0;
uint32_t i6 = 0;
uint16_t i6_d = 0;
uint32_t i6_f = 0;
//
uint16_t value = 0;
uint8_t index_adc_dma = 0;




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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	if(hadc->Instance == ADC1){
		
		flag_adc_ready = 1;
		
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
  MX_ADC_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_USART1_UART_Init();
//  MX_USART2_UART_Init();
//  MX_SPI1_Init();
//  MX_ADC_Init();
//  MX_I2C2_Init();
		
//	MX_GPIO_Init();
//  MX_I2C2_Init();
//  MX_DMA_Init();
//  MX_USART1_UART_Init();
//  MX_SPI1_Init();
//  MX_ADC_Init();

	
	
	//------------------------------------------
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc, 9);
	//HAL_ADC_Start_IT(&hadc);  
	
	
	HAL_UART_Receive_IT( &huart1, (uint8_t*) &data_uart_receive, 1);
	
	HAL_Delay(100);
	
  Init_lcd_1602(&hi2c2);
  lcd_led_on();
  char mas_char[21];
  int count=0;
	
	mid_value = 4096/2;

	//------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  {
		
		if( flag_adc_ready ){
			
				HAL_ADC_Stop_DMA(&hadc); // не об�?з�?тельно
			
				//copy adc to adc temp bufer, adc set zero;
				for( uint8_t i = 0; i<10; i++){
					data_adc[i] = adc[i];
					adc[i] = 0;
				}
					
				//detect min max
				value = data_adc[0];
				if (value > last_value_max_u1) last_value_max_u1 = value;
				if (value < last_value_min_u1) last_value_min_u1 = value;
				value = data_adc[1];
				if (value > last_value_max_u2) last_value_max_u2 = value;
				if (value < last_value_min_u2) last_value_min_u2 = value;
				value = data_adc[2];
				if (value > last_value_max_u3) last_value_max_u3 = value;
				if (value < last_value_min_u3) last_value_min_u3 = value;
				value = data_adc[3];
				if (value > last_value_max_u4) last_value_max_u4 = value;
				if (value < last_value_min_u4) last_value_min_u4 = value;
				value = data_adc[4];
				if (value > last_value_max_u5) last_value_max_u5 = value;
				if (value < last_value_min_u5) last_value_min_u5 = value;
				value = data_adc[5];
				if (value > last_value_max_u6) last_value_max_u6 = value;
				if (value < last_value_min_u6) last_value_min_u6 = value;
			
			
				value = data_adc[6];
				if (value > last_value_max_i1) last_value_max_i1 = value;
				if (value < last_value_min_i1) last_value_min_i1 = value;			
				value = data_adc[7];
				if (value > last_value_max_i2) last_value_max_i2 = value;
				if (value < last_value_min_i2) last_value_min_i2 = value;			
				value = data_adc[8];
				if (value > last_value_max_i3) last_value_max_i3 = value;
				if (value < last_value_min_i3) last_value_min_i3 = value;			
//				value = data_adc[3];
//				if (value > last_value_max_i4) last_value_max_i4 = value;
//				if (value < last_value_min_i4) last_value_min_i4 = value;			
//				value = data_adc[4];
//				if (value > last_value_max_i5) last_value_max_i5 = value;
//				if (value < last_value_min_i5) last_value_min_i5 = value;			
//				value = data_adc[5];
//				if (value > last_value_max_i6) last_value_max_i6 = value;
//				if (value < last_value_min_i6) last_value_min_i6 = value;
				
				
				//  intetval > 40ms ?
				if( ( HAL_GetTick() - time_adc_u ) > 40 ){ // intetval 40ms
				
				
						//	u1	-----------------------------------------------------------------
						max_filtered_u1 = (1 - 0.2) * max_filtered_u1 + 0.2 * last_value_max_u1;
						min_filtered_u1 = (1 - 0.2) * min_filtered_u1 + 0.2 * last_value_min_u1;
					
						last_value_max_u1 = mid_value;
						last_value_min_u1 = mid_value;
					
						u1 = max_filtered_u1 - min_filtered_u1;	//amplitude
						u1 = u1 * 100;
						u1 = u1/910;
						if( u1 > 1 && u1 < 50){
								u1 = u1 - 1;
						}
						if(u1 < 10) u1=0;
						else u1 = u1 - 1;
				
						//	u2	-----------------------------------------------------------------
						max_filtered_u2 = (1 - 0.2) * max_filtered_u2 + 0.2 * last_value_max_u2;
						min_filtered_u2 = (1 - 0.2) * min_filtered_u2 + 0.2 * last_value_min_u2;
					
						last_value_max_u2 = mid_value;
						last_value_min_u2 = mid_value;
					
						u2 = max_filtered_u2 - min_filtered_u2;	//amplitude
						u2 = u2 * 100;
						u2 = u2/910;
						if( u2 > 0 && u2 < 50){
								u2 = u2 - 1;
						}
						if(u2 < 10) u2=0;
						else u2 = u2 + 3;
				
						//	u3	-----------------------------------------------------------------
						max_filtered_u3 = (1 - 0.2) * max_filtered_u3 + 0.2 * last_value_max_u3;
						min_filtered_u3 = (1 - 0.2) * min_filtered_u3 + 0.2 * last_value_min_u3;
					
						last_value_max_u3 = mid_value;
						last_value_min_u3 = mid_value;
					
						u3 = max_filtered_u3 - min_filtered_u3;	//amplitude
						u3 = u3 * 100;
						u3 = u3/910;
						if( u3 > 0 && u3 < 50){
								u3 = u3 - 1;
						}
						if(u3 < 10) u3=0;
						else u3 = u3 - 0;
				
						//	u4	-----------------------------------------------------------------
						max_filtered_u4 = (1 - 0.2) * max_filtered_u4 + 0.2 * last_value_max_u4;
						min_filtered_u4 = (1 - 0.2) * min_filtered_u4 + 0.2 * last_value_min_u4;
					
						last_value_max_u4 = mid_value;
						last_value_min_u4 = mid_value;
					
						u4 = max_filtered_u4 - min_filtered_u4;	//amplitude
						u4 = u4 * 100;
						u4 = u4/910;
						if( u4 > 0 && u4 < 50){
								u4 = u4 - 1;
						}
						if(u4 < 10) u4=0;
						else u4 = u4 + 1;
				
						//	u5	-----------------------------------------------------------------
						max_filtered_u5 = (1 - 0.2) * max_filtered_u5 + 0.2 * last_value_max_u5;
						min_filtered_u5 = (1 - 0.2) * min_filtered_u5 + 0.2 * last_value_min_u5;
					
						last_value_max_u5 = mid_value;
						last_value_min_u5 = mid_value;
					
						u5 = max_filtered_u5 - min_filtered_u5;	//amplitude
						u5 = u5 * 100;
						u5 = u5/910;
						if( u5 > 0 && u5 < 50){
								u5 = u5 - 1;
						}
						if(u5 < 10) u5=0;
						else u5 = u5 + 1;
				
						//	u6	-----------------------------------------------------------------
						max_filtered_u6 = (1 - 0.2) * max_filtered_u6 + 0.2 * last_value_max_u6;
						min_filtered_u6 = (1 - 0.2) * min_filtered_u6 + 0.2 * last_value_min_u6;
					
						last_value_max_u6 = mid_value;
						last_value_min_u6 = mid_value;
					
						u6 = max_filtered_u6 - min_filtered_u6;	//amplitude
						u6 = u6 * 100;
						u6 = u6/910;			
						if( u6 > 0 && u6 < 50){
								u6 = u6 - 1;
						}
						if(u6 < 10) u6=0;
						else u6 = u6 + 1;
						
						
						
						
						
						//	i1	-----------------------------------------------------------------
						max_filtered_i1 = (1 - 0.2) * max_filtered_i1 + 0.2 * last_value_max_i1;
						min_filtered_i1 = (1 - 0.2) * min_filtered_i1 + 0.2 * last_value_min_i1;
					
						last_value_max_i1 = mid_value;
						last_value_min_i1 = mid_value;
					
						i1 = max_filtered_i1 - min_filtered_i1;	//amplitude
						//corect
						if ( i1 < 45 ) i1 = 0;  
						i1 = i1 * 100;
						i1_d = i1/18300;
						i1_f = i1%18300;
						i1_f =  i1_f * 10;
						i1_f = i1_f/18300;						
						//	i2	-----------------------------------------------------------------
						max_filtered_i2 = (1 - 0.2) * max_filtered_i2 + 0.2 * last_value_max_i2;
						min_filtered_i2 = (1 - 0.2) * min_filtered_i2 + 0.2 * last_value_min_i2;
					
						last_value_max_i2 = mid_value;
						last_value_min_i2 = mid_value;
					
						i2 = max_filtered_i2 - min_filtered_i2;	//amplitude
						//corect
						if ( i2 < 45 ) i2 = 0;  
						i2 = i2 * 100;
						i2_d = i2/18300;
						i2_f = i2%18300;
						i2_f =  i2_f * 10;
						i2_f = i2_f/18300;
												
						//	i3	-----------------------------------------------------------------
						max_filtered_i3 = (1 - 0.2) * max_filtered_i3 + 0.2 * last_value_max_i3;
						min_filtered_i3 = (1 - 0.2) * min_filtered_i3 + 0.2 * last_value_min_i3;
					
						last_value_max_i3 = mid_value;
						last_value_min_i3 = mid_value;
					
						i3 = max_filtered_i3 - min_filtered_i3;	//amplitude
						//corect
						if ( i3 < 45 ) i3 = 0;  
						i3 = i3 * 100;
						i3_d = i3/18300;
						i3_f = i3%18300;
						i3_f =  i3_f * 10;
						i3_f = i3_f/18300;
												
//						//	i4	-----------------------------------------------------------------
//						max_filtered_i4 = (1 - 0.2) * max_filtered_i4 + 0.2 * last_value_max_i4;
//						min_filtered_i4 = (1 - 0.2) * min_filtered_i4 + 0.2 * last_value_min_i4;
//					
//						last_value_max_i4 = mid_value;
//						last_value_min_i4 = mid_value;
//					
//						i4 = max_filtered_i4 - min_filtered_i4;	//amplitude
//						//corect
//						if ( i4 < 45 ) i4 = 0;  
//						i4 = i4 * 100;
//						i4_d = i4/18300;
//						i4_f = i4%18300;
//						i4_f =  i4_f * 10;
//						i4_f = i4_f/18300;
//												
//						//	i5	-----------------------------------------------------------------
//						max_filtered_i5 = (1 - 0.2) * max_filtered_i5 + 0.2 * last_value_max_i5;
//						min_filtered_i5 = (1 - 0.2) * min_filtered_i5 + 0.2 * last_value_min_i5;
//					
//						last_value_max_i5 = mid_value;
//						last_value_min_i5 = mid_value;
//					
//						i5 = max_filtered_i5 - min_filtered_i5;	//amplitude
//						//corect
//						if ( i5 < 45 ) i5 = 0;  
//						i5 = i5 * 100;
//						i5_d = i5/18300;
//						i5_f = i5%18300;
//						i5_f =  i5_f * 10;
//						i5_f = i5_f/18300;
//												
//						//	i6	-----------------------------------------------------------------
//						max_filtered_i6 = (1 - 0.2) * max_filtered_i6 + 0.2 * last_value_max_i6;
//						min_filtered_i6 = (1 - 0.2) * min_filtered_i6 + 0.2 * last_value_min_i6;
//					
//						last_value_max_i6 = mid_value;
//						last_value_min_i6 = mid_value;
//					
//						i6 = max_filtered_i6 - min_filtered_i6;	//amplitude
//						//corect
//						if ( i6 < 45 ) i6 = 0;  
//						i6 = i6 * 100;
//						i6_d = i6/18300;
//						i6_f = i6%18300;
//						i6_f =  i6_f * 10;
//						i6_f = i6_f/18300;
//						
						
						//
						//sprintf(mas_char,"%4uv  %2u,%02u", u1, i1_d, i1_f);
						//
						
						//
						sprintf(mas_char,"%3u   %3u   %3u", u1, u2, u3);
						Lcd_1602_SetPos(&hi2c2, 0, 0);
						Lcd_1602_Write_Data(&hi2c2, (uint8_t *)mas_char);
						//
						//
						sprintf(mas_char,"%3u   %3u   %3u", u4, u5, u6);
						Lcd_1602_SetPos(&hi2c2, 0, 1);
						Lcd_1602_Write_Data(&hi2c2, (uint8_t *)mas_char);
						//
						
						//
						sprintf(mas_char,"%2u,%01u %2u,%01u %2u,%01u ", i1_d, i1_f, i2_d, i2_f, i3_d, i3_f);
						Lcd_1602_SetPos(&hi2c2, 0, 2);
						Lcd_1602_Write_Data(&hi2c2, (uint8_t *)mas_char);
						//
//						//
//						sprintf(mas_char,"%2u,%01u %2u,%01u %2u,%01u ", i4_d, i4_f, i5_d, i5_f, i6_d, i6_f);
//						Lcd_1602_SetPos(&hi2c2, 0, 3);
//						Lcd_1602_Write_Data(&hi2c2, (uint8_t *)mas_char);
//						//
						
						time_adc_u = HAL_GetTick();		//next 40ms
						
						
				}
				

			flag_adc_ready = 0;
			//HAL_ADC_Start_IT(&hadc);	//запу�?тим новое аналогово-цифровое преобразование
			HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc, 9);	//запуѿтим новое аналогово-цифровое преобразование
		}			
		
		
		
		if( ( HAL_GetTick() - time ) > 100 ){ // intetval 1000ms = 1s
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			
				time = HAL_GetTick();
		}
		
		

		
		
		if( ( HAL_GetTick() - time_init_uart ) > 40000 ){ // intetval 1000ms = 1s
			
				if( flag_uart_init == 0 ){
						//init sim800
						init_sim800_udp(&huart1);
					
						flag_uart_init = 1;		//do not call init next time
				}
			
				time_init_uart = HAL_GetTick();
		}
		
		
		
		if( ( HAL_GetTick() - time_send_sim800 ) > 30000 ){ // intetval 1000ms = 1s
			
				if( flag_uart_init == 1 ){
					
						sim800_send_udp_data( &huart1, u1, u2, u3, u4, u5, u6, i1_d, i1_f, i2_d, i2_f, i3_d, i3_f, i4_d, i4_f, i5_d, i5_f, i6_d, i6_f);
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
  huart2.Init.BaudRate = 38400;
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
