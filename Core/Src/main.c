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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum{
	autoSwitch,
	manualSwitch
}Switch_status;
typedef enum{
	led_red,
	led_orange,
	led_green
}Led_type;
Led_type status_led=led_green;
typedef enum{
	ProgramRunning,
	ProgramStop

}EMG_status;
EMG_status status=ProgramRunning;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
modbus_t telegram;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readvolt */
osThreadId_t readvoltHandle;
const osThreadAttr_t readvolt_attributes = {
  .name = "readvolt",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readinput */
osThreadId_t readinputHandle;
const osThreadAttr_t readinput_attributes = {
  .name = "readinput",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
modbusHandler_t ModbusH;
uint16_t ModbusDATA[8];
modbusHandler_t ModbusH2;
uint16_t ModbusDATA2[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartReadVolt(void *argument);
void StartReadInput(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Program_main(Switch_status status)
{
	if(status==autoSwitch)
	{
		if(ModbusH.u16regs[0]<2200){
			HAL_GPIO_WritePin(Y00_GPIO_Port, Y00_Pin, GPIO_PIN_SET);
			status_led=led_orange;
		}
		if(ModbusH.u16regs[0]>2800){
			HAL_GPIO_WritePin(Y00_GPIO_Port, Y00_Pin, GPIO_PIN_RESET);
			status_led=led_green;
		}


	}
	if(status==manualSwitch){
		HAL_GPIO_WritePin(Y00_GPIO_Port, Y00_Pin, GPIO_PIN_SET);
		status_led=led_orange;
	}
}
void Program_Running()
{
//	if(HAL_GPIO_ReadPin(X02_GPIO_Port, X02_Pin)==0)
	if(ModbusH2.u16regs[0]==1)
	{
		Program_main(autoSwitch);
	}
//	if(HAL_GPIO_ReadPin(X03_GPIO_Port, X03_Pin)==0)
	if(ModbusH2.u16regs[1]==1)
	{
		Program_main(manualSwitch);
	}
//	if(HAL_GPIO_ReadPin(X02_GPIO_Port, X02_Pin)==1&&HAL_GPIO_ReadPin(X03_GPIO_Port, X03_Pin)==1)
//	{
//		HAL_GPIO_WritePin(Y00_GPIO_Port, Y00_Pin, GPIO_PIN_RESET);
//	}
}
void Program_Stop()
{
	HAL_GPIO_WritePin(Y00_GPIO_Port, Y00_Pin, GPIO_PIN_RESET);
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  /* Master initialization */
  ModbusH.uModbusType = MB_MASTER;
    ModbusH.port =  &huart3;
    ModbusH.u8id = 0; // For master it must be 0
    ModbusH.u16timeOut = 1000;
    ModbusH.EN_Port = NULL;
//    ModbusH.EN_Port = EN_485_GPIO_Port;
//    ModbusH.EN_Pin = EN_485_Pin;
    ModbusH.u16regs = ModbusDATA;
    ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
    ModbusH.xTypeHW = USART_HW;
    //Initialize Modbus library
    ModbusInit(&ModbusH);
    //Start capturing traffic on serial Port
    ModbusStart(&ModbusH);



    /* Slave initialization */
    ModbusH2.uModbusType = MB_SLAVE;
      ModbusH2.port =  &huart1;
      ModbusH2.u8id = 1;
      ModbusH2.u16timeOut = 1000;
      ModbusH2.EN_Port = NULL;
      //ModbusH2.EN_Port = LD2_GPIO_Port;
      //ModbusH2.EN_Pin = LD2_Pin;
      ModbusH2.u16regs = ModbusDATA2;
      ModbusH2.u16regsize= sizeof(ModbusDATA2)/sizeof(ModbusDATA2[0]);
      //Initialize Modbus library
      ModbusH2.xTypeHW = USART_HW;
      ModbusInit(&ModbusH2);
      //Start capturing traffic on serial Port
      ModbusStart(&ModbusH2);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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

  /* creation of readvolt */
  readvoltHandle = osThreadNew(StartReadVolt, NULL, &readvolt_attributes);

  /* creation of readinput */
  readinputHandle = osThreadNew(StartReadInput, NULL, &readinput_attributes);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Y02_Pin|Y03_Pin|Y07_Pin|Y06_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Y05_Pin|Y00_Pin|Y01_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Y04_GPIO_Port, Y04_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X15_Pin X02_Pin X03_Pin */
  GPIO_InitStruct.Pin = X15_Pin|X02_Pin|X03_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : X00_Pin X01_Pin X12_Pin X14_Pin
                           X04_Pin X05_Pin */
  GPIO_InitStruct.Pin = X00_Pin|X01_Pin|X12_Pin|X14_Pin
                          |X04_Pin|X05_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Y02_Pin Y03_Pin Y07_Pin Y06_Pin */
  GPIO_InitStruct.Pin = Y02_Pin|Y03_Pin|Y07_Pin|Y06_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : X13_Pin X10_Pin X11_Pin X06_Pin
                           X07_Pin */
  GPIO_InitStruct.Pin = X13_Pin|X10_Pin|X11_Pin|X06_Pin
                          |X07_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Y05_Pin Y00_Pin Y01_Pin */
  GPIO_InitStruct.Pin = Y05_Pin|Y00_Pin|Y01_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Y04_Pin */
  GPIO_InitStruct.Pin = Y04_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Y04_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

	  if(status==ProgramRunning){
			  Program_Running();
	  }
	  else{
		  Program_Stop();
		  status_led=led_red;
	  }
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReadVolt */
/**
* @brief Function implementing the readvolt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadVolt */
void StartReadVolt(void *argument)
{
  /* USER CODE BEGIN StartReadVolt */
  /* Infinite loop */
  for(;;)
  {
	  uint32_t u32NotificationValue;
	  	    telegram.u8id = 1; // slave address
	  	    telegram.u8fct = 4; // function code (this one is registers read)
	  	    //telegram[0].u16RegAdd = 0x160; // start address in slave
	  	    telegram.u16RegAdd = 0x0; // start address in slave
	  	    telegram.u16CoilsNo = 8; // number of elements (coils or registers) to read
	  	    telegram.u16reg = ModbusDATA; // pointer to a memory array in the Arduino

	  	    int aux;

	  	    for(;;)
	  	    {
	  	  	  ModbusQuery(&ModbusH, telegram); // make a query
	  	  	  u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
	  	  	  if(u32NotificationValue != ERR_OK_QUERY)
	  	  	  {
	  	  		//handle error
	  	  		//  while(1);
	  	  		  aux =1;
	  	  	  }
	  	  	  uint16_t Volt_battery  = ModbusH.u16regs[0];
	  	  	  uint16_t Current_battery = ModbusH.u16regs[1];
	  	  	  ModbusH2.u16regs[3]=Volt_battery;

	  	    }
    osDelay(100);
  }
  /* USER CODE END StartReadVolt */
}

/* USER CODE BEGIN Header_StartReadInput */
/**
* @brief Function implementing the readinput thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadInput */
void StartReadInput(void *argument)
{
  /* USER CODE BEGIN StartReadInput */
  /* Infinite loop */
  for(;;)
  {
//	  if(HAL_GPIO_ReadPin(X00_GPIO_Port, X00_Pin)==0)
	  if(ModbusH2.u16regs[2]==1)
	  {
		  if(status==ProgramRunning)
		  {
			  status=ProgramStop;
		  }

	  }
	  if(HAL_GPIO_ReadPin(X00_GPIO_Port, X00_Pin)==1&&HAL_GPIO_ReadPin(X01_GPIO_Port, X01_Pin)==0)
	  {
		  if(status==ProgramStop)
		  {
			  status=ProgramRunning;

		  }
	  }
	  switch (status_led) {
		case led_green:
			HAL_GPIO_WritePin(Y02_GPIO_Port, Y02_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Y03_GPIO_Port, Y03_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Y04_GPIO_Port, Y04_Pin, GPIO_PIN_RESET);
			break;
		case led_orange:
			HAL_GPIO_WritePin(Y02_GPIO_Port, Y02_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Y03_GPIO_Port, Y03_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Y04_GPIO_Port, Y04_Pin, GPIO_PIN_RESET);
			break;
		case led_red:
			HAL_GPIO_WritePin(Y02_GPIO_Port, Y02_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Y03_GPIO_Port, Y03_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Y04_GPIO_Port, Y04_Pin, GPIO_PIN_SET);
			break;
		default:
			break;
	}
    osDelay(100);
  }
  /* USER CODE END StartReadInput */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
