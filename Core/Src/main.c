/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MSG_ROW_LEN   255
#define MSG_ROWS      8
#define RX_BUF_SIZE   512
#define TX_BUF_SIZE   80
#define MAIN_BUF_SIZE 2048


typedef struct
{
	uint32_t TimeStamp;
	uint16_t DataLen;
	uint8_t  Source;
	uint8_t  Data[MSG_ROW_LEN];
} MsgData_st;

typedef struct
{
	MsgData_st Row[MSG_ROWS];
	uint8_t    head_indx;
	uint8_t    tail_indx;
} MsgBook_st;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for parseNewMsgtask */
osThreadId_t parseNewMsgtaskHandle;
const osThreadAttr_t parseNewMsgtask_attributes = {
  .name = "parseNewMsgtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for txdReadySema */
osSemaphoreId_t txdReadySemaHandle;
const osSemaphoreAttr_t txdReadySema_attributes = {
  .name = "txdReadySema"
};
/* Definitions for bufferAccessSema */
osSemaphoreId_t bufferAccessSemaHandle;
const osSemaphoreAttr_t bufferAccessSema_attributes = {
  .name = "bufferAccessSema"
};
/* Definitions for availRowCntSema */
osSemaphoreId_t availRowCntSemaHandle;
const osSemaphoreAttr_t availRowCntSema_attributes = {
  .name = "availRowCntSema"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void StartNewMsgTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RxBuf1[RX_BUF_SIZE];
uint8_t RxBuf6[RX_BUF_SIZE];
//uint8_t RxBuf2[RX_BUF_SIZE];
char    TxBuf2[TX_BUF_SIZE];

//uint8_t MainBuf[MAIN_BUF_SIZE];

MsgBook_st MsgBook;
uint32_t millis = 0;
bool     uart2_tx_done;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	osStatus_t osRes;
	if (huart->Instance == USART6)
	{
		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		MsgBook.Row[MsgBook.head_indx].Source = 6;
		MsgBook.Row[MsgBook.head_indx].DataLen = Size;
		MsgBook.Row[MsgBook.head_indx].TimeStamp = millis;
		memcpy(MsgBook.Row[MsgBook.head_indx].Data, RxBuf6, Size);

		MsgBook.head_indx++;
		if (MsgBook.head_indx >= MSG_ROWS)
		{
			MsgBook.head_indx = 0;
		}
		osSemaphoreRelease(availRowCntSemaHandle);

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *) RxBuf6, RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);

	}

	if (huart->Instance == USART1)
	{
		HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);
		//oldPos = newPos;  // Update the last position before copying new data
		MsgBook.Row[MsgBook.head_indx].Source = 1;
		MsgBook.Row[MsgBook.head_indx].DataLen = Size;
		MsgBook.Row[MsgBook.head_indx].TimeStamp = millis;
		memcpy(MsgBook.Row[MsgBook.head_indx].Data, RxBuf1, Size);

		MsgBook.head_indx++;
		if (MsgBook.head_indx >= MSG_ROWS)
		{
			MsgBook.head_indx = 0;
		}

		osSemaphoreRelease(availRowCntSemaHandle);

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuf1, RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		osSemaphoreRelease(txdReadySemaHandle);
		uart2_tx_done = true;
		__HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);
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
	char txt[80];
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  memset(&MsgBook,0x00,sizeof(MsgBook));

  HAL_UART_MspInit(&huart2);
  memset(TxBuf2, 0x00, TX_BUF_SIZE);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxBuf6, RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);


  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf1, RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  //HAL_UART_Transmit_DMA(&huart2, TxBuf2, TX_BUF_SIZE);

  //osRes = osSemaphoreRelease(newRowSemaHandle);
  sprintf(txt,"STM32Tx_Analyzer Blackpill \r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of txdReadySema */
  txdReadySemaHandle = osSemaphoreNew(1, 1, &txdReadySema_attributes);

  /* creation of bufferAccessSema */
  bufferAccessSemaHandle = osSemaphoreNew(1, 1, &bufferAccessSema_attributes);

  /* creation of availRowCntSema */
  availRowCntSemaHandle = osSemaphoreNew(2, 2, &availRowCntSema_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of parseNewMsgtask */
  parseNewMsgtaskHandle = osThreadNew(StartNewMsgTask, NULL, &parseNewMsgtask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  sprintf(txt,"STM32F411_Rx_Analyzer\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_WHITE_Pin */
  GPIO_InitStruct.Pin = LED_WHITE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_WHITE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartNewMsgTask */
/**
* @brief Function implementing the parseNewMsgtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNewMsgTask */
void StartNewMsgTask(void *argument)
{
  /* USER CODE BEGIN StartNewMsgTask */
	char txt[40];
	osStatus_t osRes;
	sprintf(txt,"Parse 1 \r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);

	osRes = osSemaphoreAcquire(availRowCntSemaHandle, osWaitForever);  // Wait for new messages
	//osSemaphoreRelease(bufferAccessSemaHandle);
	osSemaphoreRelease(txdReadySemaHandle);

	//sprintf(txt,"Parse 2 \r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);

	while( osSemaphoreGetCount(availRowCntSemaHandle) > 0)
	{
		osSemaphoreAcquire(availRowCntSemaHandle,0);
	}

	sprintf(txt,"Parse 3 \r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);

  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreGetCount(availRowCntSemaHandle) > 0)
	  //if (msg_tail_row != msg_head_row )
	  {
		  osRes = osSemaphoreAcquire(availRowCntSemaHandle, osWaitForever);

		  //sprintf(txt,"Parse 4 \r\n");
		//HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);

		  osRes = osSemaphoreAcquire(txdReadySemaHandle, osWaitForever);
		  if (osRes == osOK)
		  //if(MsgBook[msg_tail_row].DataLen > 0 )
		  {

			  snprintf(TxBuf2,254,"%lu [%d] %s",
					  MsgBook.Row[MsgBook.tail_indx].TimeStamp,
					  MsgBook.Row[MsgBook.tail_indx].Source,
					  MsgBook.Row[MsgBook.tail_indx].Data);
			  //snprintf(TxBuf2,16,"ABCDEGH %d\r\n",msg_cntr++);

			  uart2_tx_done = false;
			  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)TxBuf2, strlen(TxBuf2));
			  //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)MsgBook[msg_tail_row].Data, MsgBook[msg_tail_row].DataLen);
			  __disable_irq();
			  MsgBook.Row[MsgBook.tail_indx].DataLen = 0;
			  if (++MsgBook.tail_indx >= MSG_ROWS)  MsgBook.tail_indx = 0;
			  __enable_irq();


			  while ( !uart2_tx_done )
			  {
				  osDelay(10);
			  }

			  /*
			  HAL_UART_Transmit_IT(&huart2, TxBuf2, sizeof (txt));
			  HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);
			  HAL_UART_Transmit_IT(&huart2, (uint8_t*)MsgBook[msg_tail_row].Data, MsgBook[msg_tail_row].DataLen);
			  */
		  }
	  }

  }
  /* USER CODE END StartNewMsgTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
