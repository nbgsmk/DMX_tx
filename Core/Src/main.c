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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "usbd_cdc_if.h"
#include "DMX512.h"

#include "BoardSUP.h"
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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskHeartbeat */
osThreadId_t taskHeartbeatHandle;
const osThreadAttr_t taskHeartbeat_attributes = {
  .name = "taskHeartbeat",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskCdcTx */
osThreadId_t taskCdcTxHandle;
const osThreadAttr_t taskCdcTx_attributes = {
  .name = "taskCdcTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskCdcRx */
osThreadId_t taskCdcRxHandle;
const osThreadAttr_t taskCdcRx_attributes = {
  .name = "taskCdcRx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task05 */
osThreadId_t task05Handle;
const osThreadAttr_t task05_attributes = {
  .name = "task05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task06 */
osThreadId_t task06Handle;
const osThreadAttr_t task06_attributes = {
  .name = "task06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* Definitions for myTimer02 */
osTimerId_t myTimer02Handle;
const osTimerAttr_t myTimer02_attributes = {
  .name = "myTimer02"
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* Definitions for myMutex02 */
osMutexId_t myMutex02Handle;
const osMutexAttr_t myMutex02_attributes = {
  .name = "myMutex02"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* Definitions for myBinarySem02 */
osSemaphoreId_t myBinarySem02Handle;
const osSemaphoreAttr_t myBinarySem02_attributes = {
  .name = "myBinarySem02"
};
/* Definitions for myCountingSem01 */
osSemaphoreId_t myCountingSem01Handle;
const osSemaphoreAttr_t myCountingSem01_attributes = {
  .name = "myCountingSem01"
};
/* Definitions for myEvent01 */
osEventFlagsId_t myEvent01Handle;
const osEventFlagsAttr_t myEvent01_attributes = {
  .name = "myEvent01"
};
/* USER CODE BEGIN PV */
uint8_t dmxTxPayload[512];
uint8_t dmxTxPacket[513];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void taskHeartbeatStart(void *argument);
void taskCdcTxStart(void *argument);
void taskCdcRxStart(void *argument);
void task05Start(void *argument);
void task06Start(void *argument);
void Timer01Callback(void *argument);
void Timer02Callback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initDmxTxPacket(){
	// zero everything
    for (int i = 0; i < sizeof(dmxTxPacket); ++i) {
    	dmxTxPacket[i] = 0x00;
    }

    // construct start sequence
    uint32_t t = 0;				// absolute position
	uint32_t i = 0;				// relative bit counter
    for (i = 0;	i < 24; ++i)	{ dmxTxPacket[i] = 0x00;   }; t = t+i;	// >22 bit: packet start sequence
    for (i = 0;	i <  4; ++i)	{ dmxTxPacket[i] = 0xff;   }; t = t+i;	// >2 bit: packet start sequence
    for (i = 0;	i <  1; ++i)	{ dmxTxPacket[i] = 0x00;   }; t = t+i;	// 1 start bit is logic zero
    for (i = 0;	i <  8; ++i)	{ dmxTxPacket[i] = 0x00;   }; t = t+i;	// 8 bit: frame 0 is always zero for lighting applications. Otherwise the dmx lig fixture will reject it.
    for (i = 0;	i <  2; ++i)	{ dmxTxPacket[i] = 0xff;   }; t = t+i;	// 2 bit: stop bits are logic 1

    // the rest of bytes remain for user payload
}

void initDmxTxPayload(){
	// zero everything
    for (int i = 0; i < sizeof(dmxTxPayload); ++i) {
    	dmxTxPayload[i] = 0x00;
    }
}

void setDmxData(uint16_t dmxAddr, uint8_t dmxData){
	dmxTxPayload[dmxAddr] = dmxData;
}

void sendDmxFrame(){


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
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

  /* creation of myMutex02 */
  myMutex02Handle = osMutexNew(&myMutex02_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* creation of myBinarySem02 */
  myBinarySem02Handle = osSemaphoreNew(1, 1, &myBinarySem02_attributes);

  /* creation of myCountingSem01 */
  myCountingSem01Handle = osSemaphoreNew(2, 0, &myCountingSem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Timer01Callback, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* creation of myTimer02 */
  myTimer02Handle = osTimerNew(Timer02Callback, osTimerPeriodic, NULL, &myTimer02_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* creation of myQueue02 */
  myQueue02Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue02_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of taskHeartbeat */
  taskHeartbeatHandle = osThreadNew(taskHeartbeatStart, NULL, &taskHeartbeat_attributes);

  /* creation of taskCdcTx */
  taskCdcTxHandle = osThreadNew(taskCdcTxStart, NULL, &taskCdcTx_attributes);

  /* creation of taskCdcRx */
  taskCdcRxHandle = osThreadNew(taskCdcRxStart, NULL, &taskCdcRx_attributes);

  /* creation of task05 */
  task05Handle = osThreadNew(task05Start, NULL, &task05_attributes);

  /* creation of task06 */
  task06Handle = osThreadNew(task06Start, NULL, &task06_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of myEvent01 */
  myEvent01Handle = osEventFlagsNew(&myEvent01_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_0_GPIO_Port, BOARD_LED_0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BOARD_LED_0_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOARD_KEY_0_Pin */
  GPIO_InitStruct.Pin = BOARD_KEY_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_KEY_0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void USB_CDC_RxHandler_z(uint8_t *Buf, uint32_t Len) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(taskHeartbeatHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	CDC_Transmit_FS(Buf, Len);

}

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_taskHeartbeatStart */
/**
* @brief Function implementing the taskHeartbeat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskHeartbeatStart */
void taskHeartbeatStart(void *argument)
{
  /* USER CODE BEGIN taskHeartbeatStart */
	char message[] = "DMX č!\n";
	boardLedBlinkCount(2, 50, 50);								// blink at power on
	uint32_t cekaj = 0;
  /* Infinite loop */
	osDelay(1);
	for (;;) {
		osDelay(1);

		cekaj = 5000;
		if ( boardKeyPressed() == true ) {
			cekaj = 500;
		}

		int notif = ulTaskNotifyTake(pdTRUE, cekaj);			// cekaj task notifikaciju 5 sekundi
		if (0 == notif) {
			boardLedBlinkCount(5, 1, 29);						// heartbeat 5 * (1:29 duty cycle) = 150mS smanjenim intenzitetom
			if ( boardKeyPressed() == true ) {
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
			}
		} else {
			boardLedBlink(2);									// trepni jako ako je dosla notifikacija
		}

	}
  /* USER CODE END taskHeartbeatStart */
}

/* USER CODE BEGIN Header_taskCdcTxStart */
/**
* @brief Function implementing the taskCdcTx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskCdcTxStart */
void taskCdcTxStart(void *argument)
{
  /* USER CODE BEGIN taskCdcTxStart */
	uint8_t *userData_ptr = getAllChannels();

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    CDC_Transmit_FS((uint8_t*)userData_ptr, 16);
//    for (int i = 0; i < 16; ++i) {
//    	channel_value = userData_ptr[i];
//    	txBuf_u16[2*i] = 0;
//    	txBuf_u16[2*(i+1)] = channel_value;
//	}
//    CDC_Transmit_FS(txBuf_u16, sizeof(txBuf_u16));
    osDelay(1500);

  }
  /* USER CODE END taskCdcTxStart */
}

/* USER CODE BEGIN Header_taskCdcRxStart */
/**
* @brief Function implementing the taskCdcRx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskCdcRxStart */
void taskCdcRxStart(void *argument)
{
  /* USER CODE BEGIN taskCdcRxStart */
	uint32_t del = 15000;
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
		setChannel(1, 0b10011001);
		osDelay(del);
		setChannel(7, 0b10011001);
		osDelay(del);
		setChannel(8, 0b10011001);
		osDelay(del);
		setChannel(4, 0b01101100);


  }
  /* USER CODE END taskCdcRxStart */
}

/* USER CODE BEGIN Header_task05Start */
/**
* @brief Function implementing the task05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task05Start */
void task05Start(void *argument)
{
  /* USER CODE BEGIN task05Start */

	/* Infinite loop */
  for(;;)
  {
	  osDelay(5);

  }
  /* USER CODE END task05Start */
}

/* USER CODE BEGIN Header_task06Start */
/**
* @brief Function implementing the task06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task06Start */
void task06Start(void *argument)
{
  /* USER CODE BEGIN task06Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END task06Start */
}

/* Timer01Callback function */
void Timer01Callback(void *argument)
{
  /* USER CODE BEGIN Timer01Callback */

  /* USER CODE END Timer01Callback */
}

/* Timer02Callback function */
void Timer02Callback(void *argument)
{
  /* USER CODE BEGIN Timer02Callback */

  /* USER CODE END Timer02Callback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
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
