/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/*
This firmware takes control over
10 ch ADCs
1 10mS timer
10 IO

usb cdc interface


This firmware has:
Button input de-bouncing control





*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// usb variables
extern FIFO RX_FIFO;
uint8_t incomingMessagesRxBuff[64];
uint8_t outcomingMessagesTxBuff[64];
uint8_t rXbuffPointer;
bool messageReceived;

USBD_StatusTypeDef myUsbStatus = USBD_OK;
bool txBusy = false;


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
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  boardInit();

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);


  HAL_ADC_Start_DMA(&hadc, adcResultsRaw,11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  processIOControl();


	  process5milliSecondTasks();


	  process100microSecondTasks();


	  processRxCircularBufferUnload();


	  processParse();


	  processTxRoutines();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

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
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  sConfig.Channel = ADC_CHANNEL_2;
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

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 480;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, led4_Pin|led10_Pin|led5_Pin|led6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led8_Pin|led7_Pin|led9_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, rly2_Pin|rly4_Pin|rly3_Pin|rly1_Pin
                          |led3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, rly6_Pin|rly7_Pin|rly5_Pin|led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, rly8_Pin|rly9_Pin|rly10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : btn10_Pin btn5_Pin btn9_Pin btn4_Pin
                           btn6_Pin btn2_Pin */
  GPIO_InitStruct.Pin = btn10_Pin|btn5_Pin|btn9_Pin|btn4_Pin
                          |btn6_Pin|btn2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led4_Pin led10_Pin led5_Pin led6_Pin
                           rly6_Pin rly7_Pin rly5_Pin led2_Pin */
  GPIO_InitStruct.Pin = led4_Pin|led10_Pin|led5_Pin|led6_Pin
                          |rly6_Pin|rly7_Pin|rly5_Pin|led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : btn3_Pin */
  GPIO_InitStruct.Pin = btn3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(btn3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led8_Pin rly2_Pin led7_Pin rly4_Pin
                           rly3_Pin rly1_Pin led3_Pin led9_Pin */
  GPIO_InitStruct.Pin = led8_Pin|rly2_Pin|led7_Pin|rly4_Pin
                          |rly3_Pin|rly1_Pin|led3_Pin|led9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : btn7_Pin btn8_Pin */
  GPIO_InitStruct.Pin = btn7_Pin|btn8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : rly8_Pin rly9_Pin rly10_Pin led1_Pin */
  GPIO_InitStruct.Pin = rly8_Pin|rly9_Pin|rly10_Pin|led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : btn1_Pin */
  GPIO_InitStruct.Pin = btn1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(btn1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void boardInit(void){


	firmwareRev = 0.1;


	portArray[0].gpioPortButton = btn1_GPIO_Port;
	portArray[0].gpioButton = btn1_Pin;
	portArray[0].gpioPortLed = led1_GPIO_Port;
	portArray[0].gpioLed = led1_Pin;
	portArray[0].gpioPortRelay = rly1_GPIO_Port;
	portArray[0].gpioRelay = rly1_Pin;


	portArray[1].gpioPortButton = btn2_GPIO_Port;
	portArray[1].gpioButton = btn2_Pin;
	portArray[1].gpioPortLed = led2_GPIO_Port;
	portArray[1].gpioLed = led2_Pin;
	portArray[1].gpioPortRelay = rly2_GPIO_Port;
	portArray[1].gpioRelay = rly2_Pin;


	portArray[2].gpioPortButton = btn3_GPIO_Port;
	portArray[2].gpioButton = btn3_Pin;
	portArray[2].gpioPortLed = led3_GPIO_Port;
	portArray[2].gpioLed = led3_Pin;
	portArray[2].gpioPortRelay = rly3_GPIO_Port;
	portArray[2].gpioRelay = rly3_Pin;


	portArray[3].gpioPortButton = btn4_GPIO_Port;
	portArray[3].gpioButton = btn4_Pin;
	portArray[3].gpioPortLed = led4_GPIO_Port;
	portArray[3].gpioLed = led4_Pin;
	portArray[3].gpioPortRelay = rly4_GPIO_Port;
	portArray[3].gpioRelay = rly4_Pin;


	portArray[4].gpioPortButton = btn5_GPIO_Port;
	portArray[4].gpioButton = btn5_Pin;
	portArray[4].gpioPortLed = led5_GPIO_Port;
	portArray[4].gpioLed = led5_Pin;
	portArray[4].gpioPortRelay = rly5_GPIO_Port;
	portArray[4].gpioRelay = rly5_Pin;


	portArray[5].gpioPortButton = btn6_GPIO_Port;
	portArray[5].gpioButton = btn6_Pin;
	portArray[5].gpioPortLed = led6_GPIO_Port;
	portArray[5].gpioLed = led6_Pin;
	portArray[5].gpioPortRelay = rly6_GPIO_Port;
	portArray[5].gpioRelay = rly6_Pin;


	portArray[6].gpioPortButton = btn7_GPIO_Port;
	portArray[6].gpioButton = btn7_Pin;
	portArray[6].gpioPortLed = led7_GPIO_Port;
	portArray[6].gpioLed = led7_Pin;
	portArray[6].gpioPortRelay = rly7_GPIO_Port;
	portArray[6].gpioRelay = rly7_Pin;


	portArray[7].gpioPortButton = btn8_GPIO_Port;
	portArray[7].gpioButton = btn8_Pin;
	portArray[7].gpioPortLed = led8_GPIO_Port;
	portArray[7].gpioLed = led8_Pin;
	portArray[7].gpioPortRelay = rly8_GPIO_Port;
	portArray[7].gpioRelay = rly8_Pin;


	portArray[8].gpioPortButton = btn9_GPIO_Port;
	portArray[8].gpioButton = btn9_Pin;
	portArray[8].gpioPortLed = led9_GPIO_Port;
	portArray[8].gpioLed = led9_Pin;
	portArray[8].gpioPortRelay = rly9_GPIO_Port;
	portArray[8].gpioRelay = rly9_Pin;


	portArray[9].gpioPortButton = btn10_GPIO_Port;
	portArray[9].gpioButton = btn10_Pin;
	portArray[9].gpioPortLed = led10_GPIO_Port;
	portArray[9].gpioLed = led10_Pin;
	portArray[9].gpioPortRelay = rly10_GPIO_Port;
	portArray[9].gpioRelay = rly10_Pin;


	int i;


	// todo; make this variable configurable and to be read of the flash
	boardInitOfStart = false;

	if (boardInitOfStart){

		// todo: here we init the board based on stored in flash settings - post recovery state

	}else {





		for (i=0; i<10; i++){


			portArray[i].btn = GPIO_PIN_RESET;
			portArray[i].led = GPIO_PIN_SET; // LEDs are active low
			portArray[i].rly = GPIO_PIN_RESET;



			HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_SET);
			portArray[i].adcPortMax = 0;
			portArray[i].adcPortMin = 0x0FFF;

		}

	}

	// this is for parser
	plotCounter = 11;


	// TODO: tbc with all the port to initiate

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim1 ){
	  timer1Elapsed = true;
  }

  if (htim == &htim2 ){
	  timer2Elapsed = true;
  }

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	// Conversion Complete & DMA Transfer Complete As Well

	// pa0 is ADC ch0 - isn't connected
	//portArray[0].adcResultsLatched =  adcResultsRaw[0] & 0xFFFF;

	portArray[0].adcResultsLatched =  ( adcResultsRaw[0] >> 16 ) & 0xFFFF ;

	portArray[1].adcResultsLatched =  adcResultsRaw[1] & 0xFFFF;

	portArray[2].adcResultsLatched =  ( adcResultsRaw[1] >> 16 ) & 0xFFFF ;

	portArray[3].adcResultsLatched =  adcResultsRaw[2] & 0xFFFF;

	portArray[4].adcResultsLatched =  ( adcResultsRaw[2] >> 16 ) & 0xFFFF ;

	portArray[5].adcResultsLatched =  adcResultsRaw[3] & 0xFFFF;

	portArray[6].adcResultsLatched =  ( adcResultsRaw[3] >> 16 ) & 0xFFFF ;

	portArray[7].adcResultsLatched =  adcResultsRaw[4] & 0xFFFF;

	portArray[8].adcResultsLatched =  ( adcResultsRaw[4] >> 16 ) & 0xFFFF ;

	portArray[9].adcResultsLatched = adcResultsRaw[5] & 0xFFFF;


	adcCompletedFlag = true;

}



void processIOControl(void){


	uint8_t i = 0;


	// pulling all button inputs - processing is done in timed function
	for (i=0; i<10; i++){

		portArray[i].btn = HAL_GPIO_ReadPin(portArray[i].gpioPortButton, portArray[i].gpioButton);

	}

	// updating relays with accordance to update requirement
	for (i=0; i<10; i++){

		if (portArray[i].portStatusChangeRequired == true){

			portArray[i].portStatusChangeRequired = false;

			portArray[i].ledFlashSequenceRequired = true; // initiates the LED sequence
			portArray[i].ledSequenceingCounter = 0;

			if (portArray[i].relayStaus == true){

				HAL_GPIO_WritePin(portArray[i].gpioPortRelay, portArray[i].gpioRelay, GPIO_PIN_SET);
				portArray[i].rly = GPIO_PIN_SET;

			} else {

				HAL_GPIO_WritePin(portArray[i].gpioPortRelay, portArray[i].gpioRelay, GPIO_PIN_RESET);
				portArray[i].rly = GPIO_PIN_RESET;

			}

		}

	}

}



void process5milliSecondTasks(void){

	if (timer1Elapsed) {

		timer1Elapsed = false;

		// TODO: if any timing issues detected consider braking this loop

		uint8_t i;

		// TASK 1 - button de-bouncing

		// scanning for inputs and updating the statuses (in software objects only)
		for (i=0; i<10; i++){


			if (portArray[i].btn == GPIO_PIN_SET){

				if (portArray[i].buttonPressedCounter++ > buttonPressTimout){

					if (portArray[i].buttonPressed == true)// this step is to avoid continuous relay switching when button is pressed
						continue;

					// updating relays accordingly to the inputs
					if (portArray[i].relayStaus == true){
						portArray[i].relayStaus = false; // turn Off if On
					}else {
						portArray[i].relayStaus = true; // turn On if Off
					}

					portArray[i].buttonPressed = true;
					portArray[i].portStatusChangeRequired = true;

				}

			} else {

				portArray[i].buttonPressedCounter = 0;
				portArray[i].buttonPressed = false;

			}

		}


		// TASK II -  led sequence
		for (i=0; i<10; i++){

			if (portArray[i].ledFlashSequenceRequired == true){

				if (portArray[i].ledSequenceingCounter == 0 ){

					if(portArray[i].relayStaus == false){// just switched to OFF - we need to turn the led off first

						//HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_SET);

					}else { // just switched to ON

						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_RESET);

						//HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_SET);

					}

				}else if (portArray[i].ledSequenceingCounter == ledSequnceToCnt1){

					if(portArray[i].relayStaus == false){// just switched to OFF

//						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_SET);
						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_RESET);
					}else { // just switched to ON
						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_SET);

//						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_RESET);

					}


				}else if(portArray[i].ledSequenceingCounter == ledSequnceToCnt2){

					if(portArray[i].relayStaus == false){// just switched to OFF

//						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_SET);

					}else { // just switched to ON
						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(portArray[i].gpioPortLed, portArray[i].gpioLed, GPIO_PIN_SET);

					}

					// end of sequence!!! reset values, check if something else is needed

					portArray[i].ledFlashSequenceRequired = false;
					portArray[i].ledSequenceingCounter = 0;



				}

				portArray[i].ledSequenceingCounter++;

			}


		}


		// TASK III - plot variables

		// plot structure : port number : current current / max current ; port statu (on or off)

		if (plotCounter<10){
			sprintf(( char *) outcomingMessagesTxBuff, "Port#%3d : %4u / %7f : %2d \n\r", plotCounter+1, portArray[plotCounter].adcResultsLatched,  portArray[plotCounter].portMaxCurrent, portArray[plotCounter].relayStaus);
			plotCounter++;
		}else if (plotCounter == 10){
			plotCounter++;
			sprintf(( char *) outcomingMessagesTxBuff, "ETX\n\r");

		}



// sprintf(( char *) outcomingMessagesTxBuff, "Value of Pi = %f", M_PI);



	}


}



void process100microSecondTasks(void){ //100exp-6

// clock = 48Mhz
// pre scaler =48
// counter = 100

// period is clock / ( pre scaler * counter) = 48000000 / (48 * 100) = 10000 = 10 KHz


	if (timer2Elapsed) {


			timer2Elapsed = false;


			if (adcCompletedFlag == true){

//				sprintf(( char *) outcomingMessagesTxBuff,"ADC timing issues\n\r");
				adcCompletedFlag = false;
				HAL_ADC_Start_DMA( &hadc , adcResultsRaw , 11) ;

			}


			int i;


			// finding current minimal and max current values processing will take place later
			for (i=0; i<10; i++){

				if ( portArray[i].adcResultsLatched > portArray[i].adcPortMax){

					portArray[i].adcResultsLatched = portArray[i].adcPortMax;

				}

				if ( portArray[i].adcResultsLatched < portArray[i].adcPortMin){

					portArray[i].adcResultsLatched = portArray[i].adcPortMin;

				}

			}


	}


}



void processRxCircularBufferUnload(void){
	/* non-blocking but may be a heavy function
	 * for better performance consider this function in a high speed timer but check execution times
	 * reads the circular buffer and fetches the commands ending with /r
	 *
	 *
	*/

	if( RX_FIFO.head == RX_FIFO.tail ) 				// when the buffer is empty, head and tail pointing on the same location
	{
		return;
	}

	else if (rXbuffPointer == 64) 					// means the message exceeds the maximum allowed message length
	{
		rXbuffPointer = 0;
		return;
	}

	else 											// there are bytes in the circular buffer
	{

		if (RX_FIFO.data[RX_FIFO.tail] != '\n' && RX_FIFO.data[RX_FIFO.tail] != '\r')
		{
			incomingMessagesRxBuff[rXbuffPointer++] = RX_FIFO.data[RX_FIFO.tail];
		}
		else // here we will get only if there was "Enter" button pushed - End Of Message
		{	// this is a perfect place for a small SM to fetch commands, update buffers and flags based on received messages
			// these are global variables to work with
			incomingMessagesRxBuff[rXbuffPointer++] = '\0'; // NULL terminator will allow us STRING operators on uint_8 buffer
			rXbuffPointer = 0;
			messageReceived = true;

		}

		RX_FIFO.tail = FIFO_INCR(RX_FIFO.tail);

	}
}



void processParse(void){

	/*

	//vocabulary
	 *
	1+ // activate port
	1- // deactivate port
	1 25.35 // set maximum current

	plot+ - start streaming all ports
	plot- - stop streaming
	status - showing all data once

	*/



	if (messageReceived){
		messageReceived = false;

		if(strncmp((const char *) incomingMessagesRxBuff, "reset", 5) == 0){
				NVIC_SystemReset();
		}

		if (strncmp((const char *) incomingMessagesRxBuff, "hello", (int)5) == 0 ) {
			sprintf(( char *) outcomingMessagesTxBuff,"go find your friends\n\r");
		  return;
		}

		if (strncmp((const char *) incomingMessagesRxBuff, "firmware", (int)7) == 0 ) {
			sprintf(( char *) outcomingMessagesTxBuff, "built:V-%f", firmwareRev);
		  return;
		}


	}else { // no action needed if no message arrived
		return;
	}




	if (strncmp((const char *) incomingMessagesRxBuff, "plot+" , 9) ==0 ) {

		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "plot-" , 9) == 0 ){

		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "stat" , 7) == 0 ){
		// todo: change stat to status
		plotCounter = 0;
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "info" , 4) == 0 ){

		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "1+" , 2) == 0 ){

		updatePortStatus(0, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "2+" , 2) == 0 ){

		updatePortStatus(1, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "3+" , 2) == 0 ){

		updatePortStatus(2, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "4+" , 2) == 0 ){

		updatePortStatus(3, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "5+" , 2) == 0 ){

		updatePortStatus(4, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "6+" , 2) == 0 ){

		updatePortStatus(5, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "7+" , 2) == 0 ){

		updatePortStatus(6, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "8+" , 2) == 0 ){

		updatePortStatus(7, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "9+" , 2) == 0 ){

		updatePortStatus(8, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "10+" , 3) == 0 ){

		updatePortStatus(9, true);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "1-" , 2) == 0 ){

		updatePortStatus(0, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "2-" , 2) == 0 ){

		updatePortStatus(1, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "3-" , 2) == 0 ){

		updatePortStatus(2, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "4-" , 2) == 0 ){

		updatePortStatus(3, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "5-" , 2) == 0 ){

		updatePortStatus(4, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "6-" , 2) == 0 ){

		updatePortStatus(5, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "7-" , 2) == 0 ){

		updatePortStatus(6, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "8-" , 2) == 0 ){

		updatePortStatus(7, false);
		return;

	}else if (strncmp((const char *) incomingMessagesRxBuff, "9-" , 2) == 0 ){

		updatePortStatus(8, false);
		return;

	}else if (strncmp( (const char *) incomingMessagesRxBuff, "10-" , 3) == 0 ){

		updatePortStatus(9, false);
		return;

	}


	// other type of messages processing "1 12.53" - this should set the max current


	char * spaceLocation, * spaceLocation2;

	// fist checking if there are more than one space
	spaceLocation  = strchr ( (const char *) incomingMessagesRxBuff, ' '); // first occurrence
	spaceLocation2 = strrchr( (const char *) incomingMessagesRxBuff, ' '); // last occurrence

	// no spacing or two spacing
	if (spaceLocation == NULL || spaceLocation2 == NULL){
		sprintf(( char *) outcomingMessagesTxBuff,"undefined input\n\r");
		return;
	}

	if (spaceLocation2 != spaceLocation){// means there are too many spaces in the string
		sprintf(( char *) outcomingMessagesTxBuff,"error:too many spaces\n\r");
		return;
	}

	// returns pointer to first area
	char * token1 = strtok(( char*)incomingMessagesRxBuff, " ");

	uint16_t outletAddress = atoi(token1);

	char * token2 = strtok(NULL, " ");

	float incomingCurrentSetting = atof(token2);

	// input data check
	if (outletAddress > 0 && outletAddress < 11) { // 1:10

		// checking the second input - represents current
		if (incomingCurrentSetting >= 0 && incomingCurrentSetting <= 15){

			updatePortMaxCurrent(outletAddress, incomingCurrentSetting);

		}else{

			sprintf(( char *) outcomingMessagesTxBuff,"error:max current is out of range of 0-15[A]\n\r");
			return;

		}

	}else{

		sprintf(( char *) outcomingMessagesTxBuff,"error:outlet address is out of range of 1:10\n\r");
		return;

	}


}



void updatePortStatus(uint8_t portNumber, bool val){
// input is 0:9

	if (portNumber > 9 || portNumber < 0){ //[10 and negative wont go. 0:9
		sprintf(( char *) outcomingMessagesTxBuff,"internal error: out of range \n\r");
		return;
	}

	// this will initiate hardware and software updating sequence
	portArray[portNumber].portStatusChangeRequired = true;
	portArray[portNumber].relayStaus = val;
	sprintf(( char *) outcomingMessagesTxBuff,"updated \n\r");

}



void updatePortMaxCurrent(uint8_t portNumber, float newMaxCurrent){


	if (portNumber > 10 || portNumber < 1){ // 11 and 0 wont go so 10:1 will go
			sprintf(( char *) outcomingMessagesTxBuff,"internal error: out of range \n\r");
			return;
		}

	portArray[portNumber-1].portMaxCurrent = newMaxCurrent;
	sprintf(( char *) outcomingMessagesTxBuff,"updated \n\r");

}



void processTxRoutines(void){
	// there is no USB TX interrupt, we have to use sending function using dummy pulling method

	if (outcomingMessagesTxBuff[0] != '\0') { //  verification.. mainly if null means transition is complete and usb TX can send another message

		txBusy = true;

		myUsbStatus = CDC_Transmit_FS( (uint8_t *) outcomingMessagesTxBuff, strlen( (const char*) outcomingMessagesTxBuff));

		if (myUsbStatus == USBD_OK){ // if status is not OK, the buffer will remain with its data and the system will try sending it again at the next iteration.
			memset(outcomingMessagesTxBuff, '\0', 64);
			txBusy = false;
		}else{
			// todo: should not be here
		}

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
