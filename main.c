/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @project        : The BARISTA Project
  ******************************************************************************
  * @author: 		: ^HH 2022
  *
  * Copyright (c) 2022 TH Luebeck.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "senseo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 64
#define TIMEOUT 0xFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

enum state_t{
	st_PowerOff,
	st_Idle,
	st_one_cup_heating,
	st_one_cup_pumping,
	st_two_cup_heating,
	st_two_cup_pumping,
	st_no_water
}state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);

/* USER CODE BEGIN PFP */
uint16_t WaterTemp(void);
uint16_t WaterLevel_ok(void);
uint16_t OnOffKey_pressed(void);
uint16_t Cup1Key_pressed(void);
uint16_t Cup2Key_pressed(void);

void LED(uint16_t stat);
void Pump(uint16_t stat);
void Heater(uint16_t stat);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t watertemp_converter(uint16_t temp)
{
	uint16_t temp_conv = 0;
	temp_conv = -6  * temp + 105;
	return temp_conv;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char buf[BUFFER_SIZE];
	int16_t time = 0;
	int count = 0;
	int count_auschalten = 0;

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
   {
	/* USER CODE BEGIN 3 */

	  switch(state)
	  {
	  	  // ---- OFF ----
		  case st_PowerOff:
		  {
			  LED(OFF);
			  Pump(OFF);
			  Heater(OFF);

			  if(OnOffKey_pressed() != OFF)
			  {
				  state = st_Idle;
			  }

			  sprintf(buf,"State:%d",state);
			  HAL_UART_Transmit(&huart2, buf, strlen(buf), TIMEOUT);
			  break;
		  }

		  // ---- IDLE ----
		  case st_Idle :
		  {
			  LED(ON);
			  Pump(OFF);
			  Heater(OFF);

			  // start Timer 3
			  time = __HAL_TIM_GET_COUNTER(&htim3);

			  // Risk: autoamtisch nach einer Minute ausschalten
			  if ( time > 4000 ) // 1s
			  {
				  count_auschalten ++;
				  __HAL_TIM_SET_COUNTER(&htim3, OFF);

			  }
			  // Risk: autoamtisch nach einer Minute ausschalten
			  if ( count_auschalten == 60 && time > 4000 ) // 1s * 60 = 60s
			  {
				  state = st_PowerOff;
			  }

			  if (OnOffKey_pressed() !=OFF)
			  {
				  state = st_PowerOff;
			  }

			  if(Cup1Key_pressed() !=OFF)
			  {
				  state = st_one_cup_heating;
			  }

			  if(Cup2Key_pressed() !=OFF)
			  {
				  state = st_two_cup_heating;
			  }

			  sprintf(buf,"State:%d",state);
			  HAL_UART_Transmit(&huart2, buf, strlen(buf), TIMEOUT);
			  break;


		  }

		  //---- NO WATER ----
		  case st_no_water :
		  {
			  LED(ON);
			  Pump(OFF);
			  Heater(OFF);

			  if (WaterLevel_ok() != OFF)
			  {
				  state = st_Idle;
			  }

			  if(OnOffKey_pressed() !=OFF)
			  {
				  state = st_PowerOff;
			  }

			  sprintf(buf,"State:%d",state);
			  HAL_UART_Transmit(&huart2, buf, strlen(buf), TIMEOUT);
			  break;
		  }

		  // ---- ONE CUP HEATING ----
		  case st_one_cup_heating:
		  {
			  LED(SLOWBLINK);

			  if (WaterLevel_ok() != ON)
			  {
				  state = st_no_water;
			  }

			  // Risk: KaM wird durch drücken einer beliebeigen Taste auf Idle verwiesen
			  if (OnOffKey_pressed() !=OFF)
			  {
				  state = st_Idle;
			  }
			  // Risk: KaM wird durch drücken einer beliebeigen Taste auf Idle verwiesen
			  if(Cup1Key_pressed() !=OFF)
			  {
				  state = st_Idle;
			  }
			  // Risk: KaM wird durch drücken einer beliebeigen Taste auf Idle verwiesen
			  if(Cup2Key_pressed() !=OFF)
			  {
				  state = st_Idle;
			  }

			  uint16_t water_temp = WaterTemp();

			  if(watertemp_converter(water_temp) < 60)
			  {
				  Heater(1);
			  }
			  if(watertemp_converter(water_temp) > 70)
			  {
				  count = 0;
				  Heater(OFF);
				  __HAL_TIM_SET_COUNTER(&htim5, OFF);
				  state = st_one_cup_pumping;
			  }

			  sprintf(buf,"Watertemp:%d °C, State:%d\r\n",watertemp_converter(water_temp),state);
			  HAL_UART_Transmit(&huart2, buf, strlen(buf), TIMEOUT);
			  break;

		  }
		  // ---- ONE CUP PUMPING ----
		  case st_one_cup_pumping:
		  {

			  // Risk: KaM wird durch drücken einer beliebeigen Taste auf Idle verwiesen
			  if (OnOffKey_pressed() !=OFF)
			  {
				  state = st_Idle;
			  }
			  // Risk: KaM wird durch drücken einer beliebeigen Taste auf Idle verwiesen
			  if(Cup1Key_pressed() != OFF)
			  {
				  state = st_Idle;
			  }
			  // Risk: KaM wird durch drücken einer beliebeigen Taste auf Idle verwiesen
			  if(Cup2Key_pressed() != OFF)
			  {
				  state = st_Idle;
			  }

			  LED(SLOWBLINK);

			  if (WaterLevel_ok() != ON)
			  {
				  state = st_no_water;
			  }
			  if (WaterLevel_ok() != OFF)
			  {

				  Pump(ON);
				  time = __HAL_TIM_GET_COUNTER(&htim5);

				  if ( time > 20000 ) // 5s
				  {
					  count ++;
					  __HAL_TIM_SET_COUNTER(&htim5, OFF);

				  }

				  if ( count== 6 && time > 20000 ) // 5s * 6 = 30s
				  {
					  state = st_Idle;
				  }
			  }
			  sprintf(buf,"Time: %d State:%d\r\n",time,state);
			  HAL_UART_Transmit(&huart2, buf, strlen(buf), TIMEOUT);
			  break;
		  }

		  // ---- TWO CUP HEATING ----
		  case st_two_cup_heating:
		  {
			  LED(FASTBLINK);

			  if (WaterLevel_ok() != ON)
			  {
				  state = st_no_water;
			  }

			  uint16_t water_temp = WaterTemp();

			  if(watertemp_converter(water_temp) < 70)
			  {
				  Heater(1);
			  }
			  if(watertemp_converter(water_temp) > 80)
			  {
				  count = 0; // set counter to 0
				  Heater(OFF);
				  __HAL_TIM_SET_COUNTER(&htim5, OFF); // set timer to 0
				  state = st_two_cup_pumping;
			  }

			  sprintf(buf,"Watertemp:%d °C, State:%d\r\n",watertemp_converter(water_temp),state);
			  HAL_UART_Transmit(&huart2, buf, strlen(buf), TIMEOUT);
			  break;
		  }
		  case st_two_cup_pumping:
		  {
			  LED(FASTBLINK);

			  if (WaterLevel_ok() != ON)
			  {
				  state = st_no_water;
			  }
			  if (WaterLevel_ok() != OFF)
			  {

				  Pump(1);
				  time = __HAL_TIM_GET_COUNTER(&htim5);

				  if ( time > 20000 ) // 5s
				  {
					  count ++;
					  __HAL_TIM_SET_COUNTER(&htim5, 0);

				  }

				  if ( count== 12 && time > 20000 ) // 5s * 12 = 60s
				  {
					  state = st_Idle;
				  }
			  }
			  sprintf(buf,"Time: %d State:%d\r\n",time,state);
			  HAL_UART_Transmit(&huart2, buf, strlen(buf), TIMEOUT);
			  break;
		  }
	  }






	/* USER CODE END 3 */
   }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 21000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 21000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PumpControl_Pin|HeaterControl_Pin|LEDControl_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TestSignal_GPIO_Port, TestSignal_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Cup2Key_Pin Cup1Key_Pin OnOffKey_Pin */
  GPIO_InitStruct.Pin = Cup2Key_Pin|Cup1Key_Pin|OnOffKey_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PumpControl_Pin HeaterControl_Pin LEDControl_Pin */
  GPIO_InitStruct.Pin = PumpControl_Pin|HeaterControl_Pin|LEDControl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TestSignal_Pin */
  GPIO_InitStruct.Pin = TestSignal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TestSignal_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//##############################################################################
// Functions of The BARISTA Project
//##############################################################################

uint16_t WaterTemp(void) {
	int i;
	uint16_t dat;

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	dat = 0 ;
	for (i=0; i<10; i++) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		dat = dat + (HAL_ADC_GetValue(&hadc1) >> 8); 	//lowest 8 bit ignored
	}
	return dat / 10;
}


uint16_t WaterLevel_ok(void) {
	int i;
	uint16_t dat;

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	dat = 0 ;
	for (i=0; i<10; i++) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		dat = dat + HAL_ADC_GetValue(&hadc1);
	} // eo for
	if ( dat > 1000 ) return 1; else return 0;
} // eo of function


uint16_t OnOffKey_pressed(void) {
	static int lastkeystate = OFF;				//is static to save state between calls

	int keystate = !HAL_GPIO_ReadPin(OnOffKey_GPIO_Port, OnOffKey_Pin);	// get key state from input pin

	if ( keystate != lastkeystate ){			// change since last call?
		lastkeystate = keystate;				// save the keystate after change
		if ( keystate ) return PRESSED;			// key changed to 'pressed'
	} // eo if ( keystate != lastkeystate )

	return NOCHANGE;
}


uint16_t Cup1Key_pressed(void) {
	static int lastkeystate = OFF;				//is static to save state between calls

	int keystate = !HAL_GPIO_ReadPin(Cup1Key_GPIO_Port, Cup1Key_Pin);	// get key state from input pin

	if ( keystate != lastkeystate ){			// change since last call?
		lastkeystate = keystate;				// save the keystate after change
		if ( keystate ) return PRESSED;			// key changed to 'pressed'
	} // eo if ( keystate != lastkeystate )

	return NOCHANGE;
}


uint16_t Cup2Key_pressed(void) {
	static int lastkeystate = OFF;				//is static to save state between calls

	int keystate = !HAL_GPIO_ReadPin(Cup2Key_GPIO_Port, Cup2Key_Pin);	// get key state from input pin

	if ( keystate != lastkeystate ){			// change since last call?
		lastkeystate = keystate;				// save the keystate after change
		if ( keystate ) return PRESSED;			// key changed to 'pressed'
	} // eo if ( keystate != lastkeystate )

	return NOCHANGE;
}


void LED(uint16_t stat) {
	static uint32_t lapcounter = 0;
	uint32_t timestamp;

	switch ( stat ) {
		case OFF: {
			HAL_GPIO_WritePin(LEDControl_GPIO_Port, LEDControl_Pin, RESET);
			break;
		} // eo case OFF:

		case ON: {
			HAL_GPIO_WritePin(LEDControl_GPIO_Port, LEDControl_Pin, SET);
			break;
		} // eo case ON:

		case SLOWBLINK: {
			timestamp = __HAL_TIM_GET_COUNTER(&htim3);
			if ( (timestamp - lapcounter) >= 4096 ) {
				HAL_GPIO_TogglePin(LEDControl_GPIO_Port, LEDControl_Pin);
				lapcounter = timestamp;
			}
			break;
		} // eo case SLOWBLINK:

		case FASTBLINK: {
			timestamp = __HAL_TIM_GET_COUNTER(&htim3);
			if ( (timestamp - lapcounter) >= 350 ) {
				HAL_GPIO_TogglePin(LEDControl_GPIO_Port, LEDControl_Pin);
				lapcounter = timestamp;
			}
			break;
		} // eo case FASTBLINK:
	} // eo switch

} // eo function

void Pump(uint16_t stat) {
	if ( stat == ON ) {
		HAL_GPIO_WritePin(PumpControl_GPIO_Port, PumpControl_Pin, SET);
	}
	else {
		HAL_GPIO_WritePin(PumpControl_GPIO_Port, PumpControl_Pin, RESET);
	}
}


void Heater(uint16_t stat) {
	if ( stat == ON ) {
		HAL_GPIO_WritePin(HeaterControl_GPIO_Port, HeaterControl_Pin, SET);
	}
	else {
		HAL_GPIO_WritePin(HeaterControl_GPIO_Port, HeaterControl_Pin, RESET);
	}
}
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
  char msg[30];
  sprintf(msg, "\nCrashed!\nReset Board\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

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
