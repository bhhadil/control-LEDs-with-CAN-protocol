/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Board 2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;
CAN_FilterTypeDef Filtercan1;
CAN_TxHeaderTypeDef pTxHeader;
uint8_t aTxData[1] ={Board};
CAN_RxHeaderTypeDef pRxHeader;
uint8_t aRxData[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
   HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_RESET);

   Filtercan1.FilterIdHigh = 0x00;
   Filtercan1.FilterIdLow = 0x00;
   Filtercan1.FilterMaskIdHigh = 0x00;
   Filtercan1. FilterMaskIdLow = 0x00;
   Filtercan1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   Filtercan1.FilterBank = 0x00;
   Filtercan1.FilterMode = CAN_FILTERMODE_IDMASK;
   Filtercan1.FilterScale = CAN_FILTERSCALE_32BIT;
   Filtercan1.FilterActivation = CAN_FILTER_ENABLE;
   Filtercan1.SlaveStartFilterBank = 14;
   if (HAL_CAN_ConfigFilter(&hcan1, &Filtercan1)!= HAL_OK)
		  {
		    Error_Handler();
		  }
  if (HAL_CAN_Start(&hcan1)!= HAL_OK)
		  {
		    Error_Handler();
		  }
  if (HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!= HAL_OK)
		  {
		    Error_Handler();
		  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 while (HAL_GPIO_ReadPin(GPIOA, USER_Pin) == GPIO_PIN_RESET );

	    HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_RESET);

	    pTxHeader.DLC=1;
	    pTxHeader.ExtId=0;
	    pTxHeader.IDE=CAN_ID_STD;
	    pTxHeader.StdId=1;
	    pTxHeader.RTR=CAN_RTR_DATA;
	    pTxHeader.TransmitGlobalTime = DISABLE;
	    // sprintf (aTxData,"%d\r\n",Board);
	 if (HAL_CAN_AddTxMessage(&hcan1,&pTxHeader,aTxData,(uint32_t *)CAN_TX_MAILBOX0)!= HAL_OK)
	     {
	    Error_Handler();
	  }
	 while (HAL_GPIO_ReadPin(GPIOA, USER_Pin)==GPIO_PIN_SET)
	         {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Pin */
  GPIO_InitStruct.Pin = USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,&pRxHeader,aRxData)!= HAL_OK)
		  {
		     Error_Handler();
		  }
	if (pRxHeader.StdId = 0x12)
	  {
	    if (aRxData[0] == 0x01)
	    {
	      /*HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_SET);*/
	      HAL_GPIO_TogglePin(GPIOD, GREEN_Pin);
	      HAL_GPIO_TogglePin(GPIOD, RED_Pin);
	      /*HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_RESET);*/
	    }
	    if (aRxData[0] == 0x02)
	    {
	      /*HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_RESET);*/
	    	HAL_GPIO_TogglePin(GPIOD, GREEN_Pin);
	    	HAL_GPIO_TogglePin(GPIOD, RED_Pin);

	    }
	    /*if (aRxData[0] == 0x03)
	    	    {
	    	      HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_SET);
	    	      HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_SET);
	    	      HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_SET);
	    	      HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_RESET);

	    	    }
	    if (aRxData[0] == 0x04)
	    	    {
	    	      HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_SET);
	    	      HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_SET);
	    	      HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_SET);
	    	      HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_SET);

	    	    }

	     if (aRxData[0] == 0x03)
	    {
	      HAL_GPIO_WritePin(GPIOD, GREEN_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOD, ORANGE_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOD, RED_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOD, BLUE_Pin, GPIO_PIN_RESET);
	    }*/
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
