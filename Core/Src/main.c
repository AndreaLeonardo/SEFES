
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
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler.h"
#include "stdio.h"
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
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
SemaphoreHandle_t mbx;
SemaphoreHandle_t mux1;
SemaphoreHandle_t mux2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}


TaskHandle_t xHandle1 = NULL;
TaskHandle_t xHandle2 = NULL;
TaskHandle_t xHandle3 = NULL;
TaskHandle_t xHandle4 = NULL;
TaskHandle_t xHandle5 = NULL;



static void taskS1( void *pvParameters ){
	printf("Task 5(sporadic) start\n");
	int startTime = xTaskGetTickCount();
	int i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	while(xSemaphoreTake(mux1, 10) == pdFALSE){}
	startTime = xTaskGetTickCount();
	i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	xSemaphoreGive(mux1);
	printf("Task 5 end\n");
}

static void task1( void *pvParameters ){
	printf("Task 1 start\n");
	while(xSemaphoreTake(mbx, 10) == pdFALSE){}
	int startTime = xTaskGetTickCount();
	int i = 0;
	while(xTaskGetTickCount() - startTime < 100){
		i++;
	}
	printf("Task 1 end\n");
}

static void task2( void *pvParameters ){
	printf("Task 2 start\n");
	int startTime = xTaskGetTickCount();
	int i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	xSemaphoreGive(mbx);
	startTime = xTaskGetTickCount();
	i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	printf("Task 2 end\n");
}

static void task3( void *pvParameters ){
	printf("Task 3 start\n");
	while(xSemaphoreTake(mux1, 10) == pdFALSE){}
	int startTime = xTaskGetTickCount();
	int i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	xSemaphoreGive(mux1);
	startTime = xTaskGetTickCount();
	i = 0;
	while(xTaskGetTickCount() - startTime < 100){
		i++;
	}
	while(xSemaphoreTake(mux2, 10) == pdFALSE){}
	startTime = xTaskGetTickCount();
	i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	xSemaphoreGive(mux2);
	printf("Task 3 end\n");
}

static void task4(void *pvParameters){
	printf("Task 4 start\n");
	vTaskPrioritySet(NULL, 3);
	while(xSemaphoreTake(mux1, 10) == pdFALSE){}
	int startTime = xTaskGetTickCount();
	int i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	xSemaphoreGive(mux1);
	vTaskPrioritySet(NULL, 2);
	startTime = xTaskGetTickCount();
	i = 0;
	while(xTaskGetTickCount() - startTime < 100){
		i++;
	}
	vTaskPrioritySet(NULL, 3);
	while(xSemaphoreTake(mux2, 10) == pdFALSE){}
	startTime = xTaskGetTickCount();
	i = 0;
	while(xTaskGetTickCount() - startTime < 20){
		i++;
	}
	xSemaphoreGive(mux2);
	vTaskPrioritySet(NULL, 2);
	printf("Task 4 end\n");
}

static void task5(void * parameters){
	if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
		BaseType_t xReturnValue = xSchedulerSporadicJobCreate( taskS1, "S1", "S1-1", pdMS_TO_TICKS( 0 ), pdMS_TO_TICKS( 1200 ) );
		if( pdFALSE == xReturnValue ) {
			printf("Sporadic job not accepted\n");
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */
	mbx = xSemaphoreCreateCounting(1,0);
	mux1 = xSemaphoreCreateCounting(1,1);
	mux2 = xSemaphoreCreateCounting(1,1);
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
  /* USER CODE BEGIN 2 */
  printf( " Hello from Freertos\r\n" );

  	vSchedulerInit();
  	//Phase, Period, WCET, Deadline
  	vSchedulerPeriodicTaskCreate(task1, "task1", configMINIMAL_STACK_SIZE, NULL, 5, &xHandle1, pdMS_TO_TICKS(0), pdMS_TO_TICKS(400), pdMS_TO_TICKS(0), pdMS_TO_TICKS(400));
  	vSchedulerPeriodicTaskCreate(task2, "task2", configMINIMAL_STACK_SIZE, NULL, 4, &xHandle2, pdMS_TO_TICKS(0), pdMS_TO_TICKS(400), pdMS_TO_TICKS(0), pdMS_TO_TICKS(400));
  	vSchedulerPeriodicTaskCreate(task3, "task3", configMINIMAL_STACK_SIZE, NULL, 3, &xHandle3, pdMS_TO_TICKS(0), pdMS_TO_TICKS(800), pdMS_TO_TICKS(0), pdMS_TO_TICKS(800));
  	vSchedulerPeriodicTaskCreate(task4, "task4", configMINIMAL_STACK_SIZE, NULL, 2, &xHandle4, pdMS_TO_TICKS(0), pdMS_TO_TICKS(1000), pdMS_TO_TICKS(0), pdMS_TO_TICKS(1000));
  	vSchedulerPeriodicTaskCreate(task5, "sporadicLauncher", configMINIMAL_STACK_SIZE, NULL, 1, &xHandle5, pdMS_TO_TICKS(0), pdMS_TO_TICKS(1200), pdMS_TO_TICKS(0), pdMS_TO_TICKS(1200));

  	vSchedulerStart();

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
