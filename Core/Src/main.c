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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define APPS_BUF_LEN 4096
const uint32_t bpsThreshold = 2000; //need to set to store the signal value which corresponds to brakes being pressed
const uint32_t bps_MIN = 400; //Below range ADC value for BPS
const uint32_t bps_MAX = 3900; //Above range ADC value for BPS
const uint32_t APPS_0_MIN = 200; //Below range ADC value for APPS_0
const uint32_t APPS_0_MAX = 2800; //Above range ADC value for APPS_0
const uint32_t APPS_1_MIN = 400; //Below range ADC value for APPS_1
const uint32_t APPS_1_MAX = 3900; //Above range ADC value for APPS_1

//#define LOOP_TIME_INTERVAL  0.001 //loop time in counts x milliseconds, 0.001 x 0.001 == 0.001s -> 10000Hz
#define LOOP_TIME_INTERVAL  100 //loop time in counts x milliseconds, 100 x 0.001 == 0.1s -> 10Hz
#define SQRT_2 1.4142
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//uint16_t apps_Buf[APPS_BUF_LEN];
uint32_t appsVal[2]; //to store APPS ADC values

uint32_t bpsVal[2]; //to store Brake Pressure Sensor values

uint8_t BMS_Current_Limit;
float DC_Max_Current = 100.0;
float AC_Max_Current = 100.0 * SQRT_2; //Peak not RMS

//CAN Message ID's
uint32_t BMS_Current_Limit_ID = 0x00000A07;
uint32_t Drive_Enable_ID = 0x00000C07;
uint32_t Set_AC_Current_ID = 0x00000107;

bool ready_to_drive = false;

bool APPS_Failure = false;
bool implausibility = false;
bool bms_Current_Limit_Ready = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static bool Ready_to_Drive(void);
static void APPS_Mapping(uint32_t *appsVal_0, uint32_t *appsVal_1,
		uint32_t apps_PP[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

char msg[256];
char msg1[256];
uint32_t current_time, time_diff, prev_time, main_loop_count;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		Error_Handler();
	}

	//Received message from BMS about current limit (Need to config. CAN filter, so VCU only accepts messages from BMS based on BMS CAN I.D)
	if (RxHeader.ExtId == BMS_Current_Limit_ID) {
		bms_Current_Limit_Ready = true;
		BMS_Current_Limit = RxData[1] / 10;
		sprintf(msg, "BMS_Current_Limit (%ld) = %d \r\n", RxHeader.ExtId,
				BMS_Current_Limit);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	} //end if
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, &appsVal[0], 1); //start the ADC for APPS 1 (Rotational Sensor) in DMA mode
	HAL_ADC_Start_DMA(&hadc2, &appsVal[1], 1); //start the ADC for APPS 2 (Linear Sensor) in DMA mode
	HAL_ADC_Start_DMA(&hadc3, &bpsVal[0], 1); //start the ADC for Brake Pressure Sensors in DMA mode

	//Start the CAN Bus
	HAL_CAN_Start(&hcan1);

//	Initialize the CAN RX Interrupt
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	//Setting Required Data Values for CAN frame
	TxHeader.DLC = 8;	//data length in bytes
//	TxHeader.IDE = CAN_ID_STD; //specify standard CAN ID
	TxHeader.IDE = CAN_ID_EXT; //specify Extended CAN ID
	TxHeader.RTR = CAN_RTR_DATA; //specifies we are sending a CAN frame
	TxHeader.TransmitGlobalTime = DISABLE;

//	 Ready to Drive check (returns true if ready and false if not ready)
	ready_to_drive = Ready_to_Drive();

	if (ready_to_drive) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}

	uint32_t apps_Pedal_Position[2]; //to store APPS Pedal Position Values (in %)
	char msg[256];
	uint32_t AC_Current_Command;
//	uint32_t ERPM_command;

	//initialize counters
	prev_time = 0;
	current_time = 0;
	time_diff = 0;
	main_loop_count = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		time_diff = current_time - prev_time; //calculate time difference

		if (time_diff >= LOOP_TIME_INTERVAL) {

			// if 100ms have elapsed since last time this condition became true,
			// then execute your program functions

			/* execute your program functions*/
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			sprintf(msg1, "BPS1 = %d \r\n BPS2 = %d \r\n",bpsVal[0],bpsVal[1]);
			APPS_Mapping(&appsVal[0], &appsVal[1], apps_Pedal_Position);

			AC_Current_Command = apps_Pedal_Position[0] / 100.0
					* AC_Max_Current;

			if (AC_Current_Command > BMS_Current_Limit * SQRT_2) {
				AC_Current_Command = BMS_Current_Limit * SQRT_2;
			} //end if

			//Sending Drive Enable CAN message
			TxHeader.ExtId = Drive_Enable_ID; //Message ID for "Drive Enable" for motor controller
//			TxHeader.ExtId = 3079; //Message ID for "Drive Enable" for motor controller

			TxData[0] = 1;
			TxData[1] = 0x00;
			TxData[2] = 0x00;
			TxData[3] = 0x00;
			TxData[4] = 0x00;
			TxData[5] = 0x00;
			TxData[6] = 0x00;
			TxData[7] = 0x00;

			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
					!= HAL_OK) {
				Error_Handler();
			} //end if

			//Sending AC Current CAN message
			TxHeader.ExtId = Set_AC_Current_ID; //Message ID for "Set AC Current" for motor controller
//			TxHeader.ExtId = 263; //Message ID for "Set ERPM" for motor controller

//			Shifting and masking bits to split the 16 - bit value into two 8 bit values to fit in the CAN data frame
			uint8_t AC_Current_Command_high_byte = (AC_Current_Command >> 8)
					& 0xFF; // shift right by 8 bits and mask with 0xFF
			uint8_t AC_Current_Command_low_byte = AC_Current_Command & 0xFF; // mask with 0xFF to get the lower 8 bits

			//AC Current (Peak not RMS)
			TxData[0] = AC_Current_Command_high_byte;
			TxData[1] = AC_Current_Command_low_byte;
			TxData[2] = 0x00;
			TxData[3] = 0x00;
			TxData[4] = 0x00;
			TxData[5] = 0x00;
			TxData[6] = 0x00;
			TxData[7] = 0x00;

			if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
					!= HAL_OK) {
				Error_Handler();
			} //end if

			sprintf(msg,
					"APPS_1 = %lu \t APPS_2 = %lu \t PP1 = %lu \t AC_Current_Command = %lu \r\n",
					appsVal[0], appsVal[1], apps_Pedal_Position[0],
					AC_Current_Command);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
			HAL_MAX_DELAY);

			prev_time = current_time; // update previous time

		} else {
			current_time = HAL_GetTick();
		}
		main_loop_count++;

	} //end infinite while loop
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = DISABLE;
	hadc3.Init.ContinuousConvMode = ENABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = ENABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 18;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0; // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Ready_to_Drive_Sound_GPIO_Port, Ready_to_Drive_Sound_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Start_Button_Pin APPS_1_SW_Pin APPS_2_SW_Pin */
	GPIO_InitStruct.Pin = Start_Button_Pin | APPS_1_SW_Pin | APPS_2_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : Ready_to_Drive_Sound_Pin */
	GPIO_InitStruct.Pin = Ready_to_Drive_Sound_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Ready_to_Drive_Sound_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : HV_Present_Pin */
	GPIO_InitStruct.Pin = HV_Present_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(HV_Present_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static bool Ready_to_Drive(void) {

	for (;;) {
		//checking if brakes are pressed, start button is pressed and HV Present at the same time
		if ((bpsVal[0] >= bpsThreshold)
				&& (!HAL_GPIO_ReadPin(Start_Button_GPIO_Port,
				Start_Button_Pin))) {

//			if ((bpsVal[0] >= bpsThreshold)
//					&& (!HAL_GPIO_ReadPin(Start_Button_GPIO_Port,
//					Start_Button_Pin))
//					&& (!HAL_GPIO_ReadPin(HV_Present_GPIO_Port, HV_Present_Pin)))

			//sound buzzer for minimum of 1 second and maximum of 3 seconds using timer

			//Method 1
			HAL_GPIO_TogglePin(Ready_to_Drive_Sound_GPIO_Port,
			Ready_to_Drive_Sound_Pin);
			HAL_Delay(2000); //sound buzzer for 2 seconds
			HAL_GPIO_TogglePin(Ready_to_Drive_Sound_GPIO_Port,
			Ready_to_Drive_Sound_Pin);

			//Method 2
			//HAL_GPIO_WritePin(Ready_to_Drive_Sound_GPIO_Port,
			//Ready_to_Drive_Sound_Pin, GPIO_PIN_SET);
			//HAL_Delay(2000); //sound buzzer for 2 seconds
			//HAL_GPIO_WritePin(Ready_to_Drive_Sound_GPIO_Port,
			//Ready_to_Drive_Sound_Pin, GPIO_PIN_RESET);

			return true;
		} //end if
		HAL_Delay(100);
	} //end for loop

	return false; //shouldn't get to here

} //end Ready_to_Drive()

static void APPS_Mapping(uint32_t *appsVal_0, uint32_t *appsVal_1,
		uint32_t apps_PP[]) {

	apps_PP[1] = 0.08849557 * (*appsVal_1) - 37.168141592;

	if (apps_PP[1] < 0) {
		apps_PP[1] = 0;
	}
	if (apps_PP[1] > 100) {
		apps_PP[1] = 100;
	}

	apps_PP[0] = 0.05405405 * (*appsVal_0) - 35.13513513;

	if (apps_PP[0] < 0) {
		apps_PP[0] = 0;
	}
	if (apps_PP[0] > 100) {
		apps_PP[0] = 100;
	}

} //end APPS_Mapping()

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
