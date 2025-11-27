/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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
#define btn10_Pin GPIO_PIN_13
#define btn10_GPIO_Port GPIOC
#define btn5_Pin GPIO_PIN_14
#define btn5_GPIO_Port GPIOC
#define led4_Pin GPIO_PIN_15
#define led4_GPIO_Port GPIOC
#define btn3_Pin GPIO_PIN_0
#define btn3_GPIO_Port GPIOF
#define led10_Pin GPIO_PIN_1
#define led10_GPIO_Port GPIOC
#define btn9_Pin GPIO_PIN_2
#define btn9_GPIO_Port GPIOC
#define led5_Pin GPIO_PIN_4
#define led5_GPIO_Port GPIOC
#define btn4_Pin GPIO_PIN_5
#define btn4_GPIO_Port GPIOC
#define led8_Pin GPIO_PIN_12
#define led8_GPIO_Port GPIOB
#define btn7_Pin GPIO_PIN_13
#define btn7_GPIO_Port GPIOB
#define rly2_Pin GPIO_PIN_14
#define rly2_GPIO_Port GPIOB
#define led7_Pin GPIO_PIN_15
#define led7_GPIO_Port GPIOB
#define btn6_Pin GPIO_PIN_6
#define btn6_GPIO_Port GPIOC
#define led6_Pin GPIO_PIN_7
#define led6_GPIO_Port GPIOC
#define rly6_Pin GPIO_PIN_8
#define rly6_GPIO_Port GPIOC
#define rly7_Pin GPIO_PIN_9
#define rly7_GPIO_Port GPIOC
#define rly8_Pin GPIO_PIN_8
#define rly8_GPIO_Port GPIOA
#define rly9_Pin GPIO_PIN_9
#define rly9_GPIO_Port GPIOA
#define rly10_Pin GPIO_PIN_10
#define rly10_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_15
#define led1_GPIO_Port GPIOA
#define rly5_Pin GPIO_PIN_10
#define rly5_GPIO_Port GPIOC
#define btn2_Pin GPIO_PIN_11
#define btn2_GPIO_Port GPIOC
#define led2_Pin GPIO_PIN_12
#define led2_GPIO_Port GPIOC
#define btn1_Pin GPIO_PIN_2
#define btn1_GPIO_Port GPIOD
#define rly4_Pin GPIO_PIN_4
#define rly4_GPIO_Port GPIOB
#define rly3_Pin GPIO_PIN_5
#define rly3_GPIO_Port GPIOB
#define rly1_Pin GPIO_PIN_6
#define rly1_GPIO_Port GPIOB
#define led3_Pin GPIO_PIN_7
#define led3_GPIO_Port GPIOB
#define btn8_Pin GPIO_PIN_8
#define btn8_GPIO_Port GPIOB
#define led9_Pin GPIO_PIN_9
#define led9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define buttonPressTimout	6 // considering running in a 5mS timer

#define ledSequnceToCnt1 	20 // considering 5mS counter is 100mS delay
#define ledSequnceToCnt2 	40 // considering 5mS counter is 100mS delay


void boardInit(void);

void processIOControl(void);

void process5milliSecondTasks(void);

void process100microSecondTasks(void);

void processRxCircularBufferUnload(void);

void processParse(void);

void updatePortStatus(uint8_t portNumber, bool val);

void updatePortMaxCurrent(uint8_t portNumber, float newMaxCurrent);

void processTxRoutines(void);

typedef struct port{

	//PHY addresses

	GPIO_TypeDef * gpioPortButton;
	uint16_t gpioButton;

	GPIO_TypeDef * gpioPortLed;
	uint16_t gpioLed;

	GPIO_TypeDef * gpioPortRelay;
	uint16_t gpioRelay;


	// hardware
	GPIO_PinState btn;
	GPIO_PinState led;
	GPIO_PinState rly;



	// software
	// buttons
	bool buttonPressed; // this is to eliminate port switching during button cont press
	uint8_t buttonPressedCounter;

	// relays
	bool relayStaus; // stores the actual status of the relay


	// leds
	bool ledFlashSequenceRequired;
	uint16_t ledSequenceingCounter;


	uint8_t ledPWMCounter;
	uint8_t ledPWMLimit;

	// general
	bool portStatusChangeRequired;


	// adc
	uint16_t adcResultsLatched;

	float adcValue;
	float portMaxCurrent;

	uint16_t adcPortMax;
	uint16_t adcPortMin;



}port;


port portArray[10];


bool timer1Elapsed, timer2Elapsed;

// parser
uint8_t plotCounter;


// adc
uint32_t adcResultsRaw[11];
bool adcCompletedFlag;


// firmware
float firmwareRev;


// stored values
// todo: storing mechanism
bool boardInitOfStart;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
