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
#include "limits.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "DMX512.h"

#include "BoardSUP.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef uint32_t MyFlags_t;
const MyFlags_t flg_UsbCDCrx_ISR	= (1U << 0);
const MyFlags_t flg_SendToHW		= (1U << 1);
const MyFlags_t flg_ERROR_SPI		= (1U << 2);
const MyFlags_t ev_InitComplete		= (1U << 3);


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
/* Definitions for receiveDmxFromPcTask */
osThreadId_t receiveDmxFromPcTaskHandle;
const osThreadAttr_t receiveDmxFromPcTask_attributes = {
  .name = "receiveDmxFromPcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for echoDmxToPcTask */
osThreadId_t echoDmxToPcTaskHandle;
const osThreadAttr_t echoDmxToPcTask_attributes = {
  .name = "echoDmxToPcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for dmxChannelsQueue */
osMessageQueueId_t dmxChannelsQueueHandle;
const osMessageQueueAttr_t dmxChannelsQueue_attributes = {
  .name = "dmxChannelsQueue"
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
/* Definitions for dmxLLandChannelMutex */
osMutexId_t dmxLLandChannelMutexHandle;
const osMutexAttr_t dmxLLandChannelMutex_attributes = {
  .name = "dmxLLandChannelMutex"
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
/* Definitions for initDoneEvent */
osEventFlagsId_t initDoneEventHandle;
const osEventFlagsAttr_t initDoneEvent_attributes = {
  .name = "initDoneEvent"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void taskHeartbeatStart(void *argument);
void task05Start(void *argument);
void task06Start(void *argument);
void StartReceiveDmxFromPcTask(void *argument);
void StartEchoDmxToPcTask(void *argument);
void Timer01Callback(void *argument);
void Timer02Callback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ***************************
// ***************************
// SWO - Serial Wire Viewer override for printf(..)
// use like this printf("chan %d \n", itm0)
// ***************************
// ***************************
int ITMi0 = 0;		// must be global vars
int ITMi1 = 0;
char ITMc0[255];
char ITMc1[255];
__attribute__((weak)) int _write(int file, char *ptr, int len) {
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    ITM_SendChar(*ptr++);
  }
  return len;
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

  /* creation of dmxLLandChannelMutex */
  dmxLLandChannelMutexHandle = osMutexNew(&dmxLLandChannelMutex_attributes);

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

  /* creation of dmxChannelsQueue */
  dmxChannelsQueueHandle = osMessageQueueNew (2048, sizeof(uint32_t), &dmxChannelsQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of taskHeartbeat */
  taskHeartbeatHandle = osThreadNew(taskHeartbeatStart, NULL, &taskHeartbeat_attributes);

  /* creation of task05 */
  task05Handle = osThreadNew(task05Start, NULL, &task05_attributes);

  /* creation of task06 */
  task06Handle = osThreadNew(task06Start, NULL, &task06_attributes);

  /* creation of receiveDmxFromPcTask */
  receiveDmxFromPcTaskHandle = osThreadNew(StartReceiveDmxFromPcTask, NULL, &receiveDmxFromPcTask_attributes);

  /* creation of echoDmxToPcTask */
  echoDmxToPcTaskHandle = osThreadNew(StartEchoDmxToPcTask, NULL, &echoDmxToPcTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of myEvent01 */
  myEvent01Handle = osEventFlagsNew(&myEvent01_attributes);

  /* creation of initDoneEvent */
  initDoneEventHandle = osEventFlagsNew(&initDoneEvent_attributes);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

	// obavesti led diodu da je nesto doslo preko usb-a
	osThreadFlagsSet(taskHeartbeatHandle, flg_UsbCDCrx_ISR);
	// i posalji primljeni karakter u queue
	for (uint32_t i = 0; i < Len; i++) {
		uint8_t rxByte = Buf[i];
		osMessageQueuePut(dmxChannelsQueueHandle, &rxByte, 0U, 0U); 	/* The timeout must be 0 in ISR context */
	}

	// ako hoces usb_cdc local echo
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

  HAL_StatusTypeDef spiStat = 0;
  clearAllChannels();											// initialize all channels to zero
  osEventFlagsSet(initDoneEventHandle, ev_InitComplete);		// signal all ready
  /* Infinite loop */
  for(;;)
  {
	  if (1==1) {
		  // test patterns if needed
		  setAllChannels(0);
//		  setChannel(01,	0b10101010);	// 170,	0xAA
		  setChannel(02,	0b00010000);	// 16,	0x10
		  setChannel(07,	0b00001011);	// 11,	0x08
		  setChannel(510,	0b00010000);	// 16,	0x10
		  setChannel(512,	0b00010000);	// 16,	0x10
	  }

	  // SPI transmits the dmx sequence repeatedly
	  osMutexAcquire(dmxLLandChannelMutexHandle, portMAX_DELAY);
	  spiStat = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)getLLPkt(), sizeof(dmxLLPkt.combined));
	  osMutexRelease(dmxLLandChannelMutexHandle);
	  if (spiStat != HAL_OK) {
		  osThreadFlagsSet(taskHeartbeatHandle, flg_ERROR_SPI);
	  }
	  osDelay(20);

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
	uint32_t waitLen = 0;
	MyFlags_t rcvdFlags;
  /* Infinite loop */
	for (;;) {
		osDelay(1);

		waitLen = boardKeyPressed() ? 500 : 5000;									// keyPressed()=true -> ubrzani blink

		// cekaj task notifikaciju 5 sekundi ili krace
		rcvdFlags = osThreadFlagsWait(flg_UsbCDCrx_ISR | flg_SendToHW, osFlagsWaitAny, waitLen);
		if (rcvdFlags == osFlagsErrorTimeout) {
			// isteklo vreme bez ikakve notifikacije -> heartbeat blink
			// heartbeat: 5x(1:29 duty cycle) = 150mS smanjenim intenzitetom
			boardLedBlinkCount(5, 1, 29);
			if ( boardKeyPressed() == true ) {
				// ako je pritisnuto dugme, dodaj i neki string na printout
				CDC_Transmit_FS((uint8_t*) message, strlen(message));
			}

		} else {
			// notifikacija druge vrste
			if (flg_UsbCDCrx_ISR == rcvdFlags)	{
				boardLedBlink(1); 			// character received from PC via USB_CDC
				// printf("%lu: triggered UsbCDCrx_ISR\n", osKernelGetTickCount());
				__NOP();
			};
			if (flg_SendToHW == rcvdFlags)		{
				boardLedBlink(1); 			// (addr,val) pair sent to dmx queue and forwarded to hardware
//				printf("%lu: hardware updated\n", osKernelGetTickCount());
				__NOP();
			};

		}

	}
  /* USER CODE END taskHeartbeatStart */
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
	  osDelay(1);

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

/* USER CODE BEGIN Header_StartReceiveDmxFromPcTask */
/**
* @brief Function implementing the receiveDmxFromPcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveDmxFromPcTask */
void StartReceiveDmxFromPcTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveDmxFromPcTask */


	// ------------------------------
	// single byte arrives into queue
	// ------------------------------
	uint8_t rx_byte;

	// --------------------------
	// finite state machine logic
	// --------------------------
	typedef enum {
		STATE_WAITING_SYNC,
		STATE_FORWARDING_DMX_DATA
	} RxState_t;
	RxState_t curState = STATE_FORWARDING_DMX_DATA;
//	const uint8_t SYNC_SEQUENCE[] = { 0xff, 0xff, 0xff, 0xff };		// raw ffff sequence
	const uint8_t SYNC_SEQUENCE[] = { 'x', 'x', 'x', 'x' };			// "xxxx" lowercase
	const uint16_t SYNC_SEQUENCE_LEN = sizeof(SYNC_SEQUENCE);
	uint16_t sync_bytes_count = 0;


	// --------------------------
	// receive buffer and counter
	// --------------------------
	uint16_t rx_payload_count = 0;
	const uint16_t RX_PAYLOAD_REQUIRED = 4;
	uint8_t rx_assembly_buffer[RX_PAYLOAD_REQUIRED];


	osStatus_t queStat;
	osEventFlagsWait(initDoneEventHandle, ev_InitComplete, osFlagsWaitAll | osFlagsNoClear, osWaitForever);	// FREEZE!! osFlagsNoClear because other tasks wait for this, too
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
		queStat = osMessageQueueGet(dmxChannelsQueueHandle, &rx_byte, NULL, osWaitForever);
		if ( queStat == osOK ) {
			switch (curState) {
			case STATE_WAITING_SYNC:
				if (rx_byte == SYNC_SEQUENCE[sync_bytes_count]) {
					sync_bytes_count++;
					if (sync_bytes_count >= SYNC_SEQUENCE_LEN) {
						curState = STATE_FORWARDING_DMX_DATA;
						printf("FSM: Sync complete. Moving to data processing.\n");
						sync_bytes_count = 0;
						rx_payload_count = 0;
					}
				} else {
					sync_bytes_count = 0;
					printf("FSM: Waiting sync. Still nothing valid received.\n");
				}
				break;


			case STATE_FORWARDING_DMX_DATA:
				if (rx_byte == SYNC_SEQUENCE[sync_bytes_count]) {
					sync_bytes_count++;
					if (sync_bytes_count == SYNC_SEQUENCE_LEN) {
						curState = STATE_FORWARDING_DMX_DATA;		// found a complete *new* sync sequence. remain in this state
						sync_bytes_count = 0;						// clear counters and restart listening
						rx_payload_count = 0;
						// no need to clear the buffer. it will be overwritten anyway
						printf("FSM: break! Clear payload buffer and start listening again.\n");
					}


				} else {
					sync_bytes_count = 0;							// discard accidental partial sync count if any
					if (rx_payload_count < RX_PAYLOAD_REQUIRED) {
						// FREEZE!! Ovaj if blok mora biti PRVI zbog rx_payload_count++ jer sledeci if zavise od njega
						rx_assembly_buffer[rx_payload_count] = rx_byte;
						rx_payload_count++;
					}
					if (rx_payload_count > RX_PAYLOAD_REQUIRED) {
						// (rx_payload_count > RX_PAYLOAD_REQUIRED) is the only remaining possibility
						// some kind of overrun occured!
						curState = STATE_FORWARDING_DMX_DATA;
						printf("FSM: break! Payload buffer overrun. Expected %d but received %d bytes. Clear and restart listening.\n", RX_PAYLOAD_REQUIRED, rx_payload_count);
						sync_bytes_count = 0;
						rx_payload_count = 0;
					}
					if (rx_payload_count == RX_PAYLOAD_REQUIRED) {
						uint16_t adr = (rx_assembly_buffer[0] << 8) | rx_assembly_buffer[1];
						uint16_t val = (rx_assembly_buffer[2] << 8) | rx_assembly_buffer[3];
						printf("%lu: FSM: valid dmx message forwarded to hardware: setChannel(ch %d, val %d), (hex: 0x%02X, 0x%02X)\n", osKernelGetTickCount(), adr, val, adr, val);
						osThreadFlagsSet(taskHeartbeatHandle, flg_SendToHW);
						uint32_t adrval = (adr << 16) | val;
						xTaskNotify(echoDmxToPcTaskHandle, adrval, eSetValueWithOverwrite);
						setChannel(adr, val);
						rx_payload_count = 0;					// restart buffer from the beginning
					}

				}
				break;


			default:
				curState = STATE_FORWARDING_DMX_DATA;
				sync_bytes_count = 0;
				rx_payload_count = 0;
				printf("%lu: FSM: unknown state! Restart listening in STATE_FORWARDING_DMX_DATA\n", osKernelGetTickCount());
				break;

			}		// end case



		} else {
			// zbog osWaitForever ovo se nikad nece desiti

		}

	}
  /* USER CODE END StartReceiveDmxFromPcTask */
}

/* USER CODE BEGIN Header_StartEchoDmxToPcTask */
/**
* @brief Function implementing the echoDmxToPcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEchoDmxToPcTask */
void StartEchoDmxToPcTask(void *argument)
{
  /* USER CODE BEGIN StartEchoDmxToPcTask */

	uint8_t *userData_ptr = getAllChannels();
	uint32_t adrval;
	enum {
		BY_byte,
		BY_w16,
		BY_w32,
		BY_str_single_chan,
		BY_w32_16chan,
		BY_str_16chan,
		BY_str_32chan,
	} ORG = BY_w32_16chan;

	osEventFlagsWait(initDoneEventHandle, ev_InitComplete, osFlagsWaitAll | osFlagsNoClear, osWaitForever);			// FREEZE!! osFlagsNoClear because other tasks wait for this, too

  /* Infinite loop */
	for (;;) {
		osDelay(1);
		xTaskNotifyWait(0x00, ULONG_MAX, &adrval, portMAX_DELAY);	// wait for combined ( (adr << 16) | val )
		printf("USB_CDC echo a: 0x%02lx\n", (unsigned long)adrval);
		switch (ORG) {
			case BY_byte:
				uint8_t buf8[4];
				buf8[0] = (uint8_t)((adrval >> 24) & 0xFF);
				buf8[1] = (uint8_t)((adrval >> 16) & 0xFF);
				buf8[2] = (uint8_t)((adrval >> 8) & 0xFF);
				buf8[3] = (uint8_t)(adrval & 0xFF);
	    		if ( CDC_Transmit_FS( buf8, sizeof(buf8) ) == USBD_BUSY ) {
					// po potrebi signaliziraj nekom neku gresku
					// glupost! ako je USBD_BUSY nikom nista, a ako nije, racunaj da je poslato. bas teska glupost!
					// U sustini, CDC_Transmit_FS neinvazivno pokusava da posalje i vraca USBD_BUSY ako je port zauzet ili USBD_OK ako js poslato
	    		}
				break;

			case BY_w16:
				uint16_t buf16[2];
				buf16[0] = (uint16_t)((adrval >> 16) & 0xFFFF);
				buf16[1] = (uint16_t)(adrval & 0xFFFF);
				if ( CDC_Transmit_FS( (uint8_t*)buf16, sizeof(buf16) ) == USBD_BUSY ) {
					// po potrebi signaliziraj nekom neku gresku
				}
				break;

			case BY_w32:
				if ( CDC_Transmit_FS((uint8_t*)&adrval, sizeof(adrval)) == USBD_BUSY ) {
					// po potrebi signaliziraj nekom neku gresku
				}
				break;

			case BY_w32_16chan:
//				for (int chnn = 0; chnn < 16; ++chnn) {
					if ( CDC_Transmit_FS((uint8_t*)&userData_ptr, 16) == USBD_BUSY ) {
						// po potrebi signaliziraj nekom neku gresku
					}
//				}
				break;

			case BY_str_single_chan:
				char strbuf[64];
				sprintf(strbuf, "ch %lu, val %lu\n", ((adrval >> 16) & 0xFFFF), (adrval & 0xFFFF) );
				if ( CDC_Transmit_FS((uint8_t*)&strbuf, strlen(strbuf)) == USBD_BUSY ) {
					// po potrebi signaliziraj nekom neku gresku
				}
				break;

			case BY_str_16chan:
				int ARRAY_SIZE = 16;
				char strbuf16[64];
				for (int chn = 0; chn < 16; ++chn) {
					char temp_num_str[5];		// Buffer for a single number + null
					sprintf(temp_num_str, "%4u", userData_ptr[chn]);	// Use sprintf to format a single number
					strcat(strbuf16, temp_num_str);		// Concatenate to the main buffer
					if (chn < ARRAY_SIZE - 1) {
						strcat(strbuf16, ",");				// Add a comma and space if it's not the last element
					}
				}
				strcat(strbuf16, "\r\n");					// add newline at the end
				if ( CDC_Transmit_FS( (uint8_t*)strbuf16, strlen(strbuf16))  == USBD_BUSY ) {
					// po potrebi signaliziraj nekom neku gresku
				}
				break;

			default:
				// greska. sta sad?
				break;
		}

	}
  /* USER CODE END StartEchoDmxToPcTask */
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
