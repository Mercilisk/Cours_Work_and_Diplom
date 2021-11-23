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
#include "queue.h"
#include "semphr.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef	struct{
	uint8_t	First_Bit;
	uint8_t	Second_Bit;
	uint8_t	Third_Bit;
	uint8_t	Fourth_Bit;
	uint8_t Fifth_Bit;
	uint8_t	Sixth_Bit;
	uint8_t	Bit_Number;
}Symbol_TypeDef;

typedef	enum{
	Point										=	1,
	Dash,
}Type_Symbol_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	FIRST_BIT								( (uint8_t) 1)
#define	SECOND_BIT								( (uint8_t) 2)
#define	THIRD_BIT								( (uint8_t) 3)
#define	FOURTH_BIT								( (uint8_t) 4)
#define	FIFTH_BIT								( (uint8_t) 5)
#define	SIXTH_BIT								( (uint8_t) 6)
#define	END_BIT									( (uint8_t) 7)

#define	TIME_DELAY_SPH 							( (TickType_t) (500 / portTICK_RATE_MS) )

#define	Offset									( (uint8_t) 48)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
USART_HandleTypeDef husart2;

/* Definitions for Event_Func */
osThreadId_t Event_FuncHandle;
const osThreadAttr_t Event_Func_attributes = {
  .name = "Event_Func",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Set_Led */
osThreadId_t Set_LedHandle;
const osThreadAttr_t Set_Led_attributes = {
  .name = "Set_Led",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Set_UART */
osThreadId_t Set_UARTHandle;
const osThreadAttr_t Set_UART_attributes = {
  .name = "Set_UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Get_UART */
osThreadId_t Get_UARTHandle;
const osThreadAttr_t Get_UART_attributes = {
  .name = "Get_UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Scan_Queue */
osMessageQueueId_t Scan_QueueHandle;
const osMessageQueueAttr_t Scan_Queue_attributes = {
  .name = "Scan_Queue"
};
/* Definitions for Print_Queue */
osMessageQueueId_t Print_QueueHandle;
const osMessageQueueAttr_t Print_Queue_attributes = {
  .name = "Print_Queue"
};
/* Definitions for Guardian_UART */
osMutexId_t Guardian_UARTHandle;
const osMutexAttr_t Guardian_UART_attributes = {
  .name = "Guardian_UART"
};
/* Definitions for Sph_Get_Btn */
osSemaphoreId_t Sph_Get_BtnHandle;
const osSemaphoreAttr_t Sph_Get_Btn_attributes = {
  .name = "Sph_Get_Btn"
};
/* Definitions for Sph_Set_Led */
osSemaphoreId_t Sph_Set_LedHandle;
const osSemaphoreAttr_t Sph_Set_Led_attributes = {
  .name = "Sph_Set_Led"
};
/* USER CODE BEGIN PV */
volatile uint32_t	ActivTaskNumbe			=	0;

uint8_t Number_Itaration					=	0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_Init(void);
void HandlerBtnClick(void *argument);
void LedBlinkTask(void *argument);
void SetUART(void *argument);
void GetUART(void *argument);

/* USER CODE BEGIN PFP */
void Get_Btn_Task(void){
	xSemaphoreGiveFromISR(Sph_Get_BtnHandle, NULL);
	xSemaphoreGiveFromISR(Sph_Set_LedHandle, NULL);
}
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
  MX_USART2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Guardian_UART */
  Guardian_UARTHandle = osMutexNew(&Guardian_UART_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sph_Get_Btn */
  Sph_Get_BtnHandle = osSemaphoreNew(4, 4, &Sph_Get_Btn_attributes);

  /* creation of Sph_Set_Led */
  Sph_Set_LedHandle = osSemaphoreNew(4, 4, &Sph_Set_Led_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Scan_Queue */
  Scan_QueueHandle = osMessageQueueNew (4, sizeof(Symbol_TypeDef), &Scan_Queue_attributes);

  /* creation of Print_Queue */
  Print_QueueHandle = osMessageQueueNew (4, sizeof(Symbol_TypeDef), &Print_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Event_Func */
  Event_FuncHandle = osThreadNew(HandlerBtnClick, NULL, &Event_Func_attributes);

  /* creation of Set_Led */
  Set_LedHandle = osThreadNew(LedBlinkTask, NULL, &Set_Led_attributes);

  /* creation of Set_UART */
  Set_UARTHandle = osThreadNew(SetUART, NULL, &Set_UART_attributes);

  /* creation of Get_UART */
  Get_UARTHandle = osThreadNew(GetUART, NULL, &Get_UART_attributes);

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_HandlerBtnClick */
/**
  * @brief  Function implementing the Event_Func thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_HandlerBtnClick */
void HandlerBtnClick(void *argument)
{
  /* USER CODE BEGIN 5 */
		Symbol_TypeDef	*Symbol						=	(Symbol_TypeDef*)malloc(
																 sizeof(Symbol_TypeDef));
		Symbol -> Bit_Number						= 	FIRST_BIT;

	/* Infinite loop */

	/*	Очистка листа семофоров - неправельная конфигурация CMSIS V2 */
	xSemaphoreTake(Sph_Get_BtnHandle, portMAX_DELAY);
	xSemaphoreTake(Sph_Get_BtnHandle, portMAX_DELAY);
	xSemaphoreTake(Sph_Get_BtnHandle, portMAX_DELAY);
	xSemaphoreTake(Sph_Get_BtnHandle, portMAX_DELAY);

	for(;;)
	{
		/*	Обнулить данные в битах, для более простой передачи через UART	*/
		Symbol -> First_Bit			=	(uint8_t) 0;
		Symbol -> Second_Bit		=	(uint8_t) 0;
		Symbol -> Third_Bit			=	(uint8_t) 0;
		Symbol -> Fourth_Bit		=	(uint8_t) 0;
		Symbol -> Fifth_Bit			=	(uint8_t) 0;
		Symbol -> Sixth_Bit			=	(uint8_t) 0;

		xSemaphoreTake(Sph_Get_BtnHandle, portMAX_DELAY);

Point_Handling_Second_Bit:

		ActivTaskNumbe	=	1;

		/*	Детектирование нажатия как точки	*/
		if (xSemaphoreTake(Sph_Get_BtnHandle, TIME_DELAY_SPH) == pdFALSE)
		{
			switch(Symbol -> Bit_Number)
			{
				case FIRST_BIT:
				{
					Symbol -> First_Bit			=	Point;
					Symbol -> Bit_Number		= 	SECOND_BIT;
				}break;
				case SECOND_BIT:
				{
					Symbol -> Second_Bit		=	Point;
					Symbol -> Bit_Number		= 	THIRD_BIT;
				}break;
				case THIRD_BIT:
				{
					Symbol -> Third_Bit			=	Point;
					Symbol -> Bit_Number		= 	FOURTH_BIT;
				}break;
				case FOURTH_BIT:
				{
					Symbol -> Fourth_Bit		=	Point;
					Symbol -> Bit_Number		= 	FIFTH_BIT;
				}break;
				case FIFTH_BIT:
				{
					Symbol -> Fifth_Bit			=	Point;
					Symbol -> Bit_Number		= 	SIXTH_BIT;
				}break;
				case SIXTH_BIT:
				{
					Symbol -> Sixth_Bit			=	Point;
					Symbol -> Bit_Number		= 	END_BIT;
				}break;
				default:
				{
				};
			}
		}
		/*	Детектирование нажатия как тире	*/
		else
		{
			switch(Symbol -> Bit_Number)
			{
				case FIRST_BIT:
				{
					Symbol -> First_Bit			=	Dash;
					Symbol -> Bit_Number		= 	SECOND_BIT;
				}break;
				case SECOND_BIT:
				{
					Symbol -> Second_Bit		=	Dash;
					Symbol -> Bit_Number		= 	THIRD_BIT;
				}break;
				case THIRD_BIT:
				{
					Symbol -> Third_Bit			=	Dash;
					Symbol -> Bit_Number		= 	FOURTH_BIT;
				}break;
				case FOURTH_BIT:
				{
					Symbol -> Fourth_Bit		=	Dash;
					Symbol -> Bit_Number		= 	FIFTH_BIT;
				}break;
				case FIFTH_BIT:
				{
					Symbol -> Fifth_Bit			=	Dash;
					Symbol -> Bit_Number		= 	SIXTH_BIT;
				}break;
				case SIXTH_BIT:
				{
					Symbol -> Sixth_Bit			=	Dash;
					Symbol -> Bit_Number		= 	END_BIT;
				}break;
				default:
				{
				};
			}
		}

		/*	Обработка пропуска между символами	*/
		switch(xSemaphoreTake(Sph_Get_BtnHandle, 2 * TIME_DELAY_SPH))
		{
			case pdFALSE:
			{
				/*	Перемещение данных в очередь, для отправки в UART	*/
				xQueueSendToBack(Print_QueueHandle, Symbol, portMAX_DELAY);
			}break;
			default:
			{
				/*	Переход в точку Point_Handling_Second_Bit, для
				 * пропускапервлго семофора	*/
				goto Point_Handling_Second_Bit;
			};
		}
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LedBlinkTask */
/**
* @brief Function implementing the Set_Led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedBlinkTask */
void LedBlinkTask(void *argument)
{
  /* USER CODE BEGIN LedBlinkTask */

		/*	Для указания, что произошло переключение задач.
		Number_Itaration				=	2u;*/

	/*	Очистка листа семофоров - неправельная конфигурация CMSIS V2 */
	xSemaphoreTake(Sph_Set_LedHandle, portMAX_DELAY);
	xSemaphoreTake(Sph_Set_LedHandle, portMAX_DELAY);
	xSemaphoreTake(Sph_Set_LedHandle, portMAX_DELAY);
	xSemaphoreTake(Sph_Set_LedHandle, portMAX_DELAY);

	/* Infinite loop */
	for(;;)
	{
		xSemaphoreTake(Sph_Set_LedHandle, portMAX_DELAY);
		ActivTaskNumbe	=	2;
			HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET );
			osDelay(TIME_DELAY_SPH);
			HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET );
	}
  /* USER CODE END LedBlinkTask */
}

/* USER CODE BEGIN Header_SetUART */
/**
* @brief Function implementing the Set_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SetUART */
void SetUART(void *argument)
{
  /* USER CODE BEGIN SetUART */
	Symbol_TypeDef *Transmit_Symbol				=	(Symbol_TypeDef*)malloc(
			 	 	 	 	 	 	 	 	 	 	 	 	 	 	 sizeof(Symbol_TypeDef));;
	uint8_t	Sumbol_Bits[6] = {0, 0, 0, 0, 0, 0};
	/* Infinite loop */
	for(;;)
	{
		ActivTaskNumbe	=	3;
		xQueueReceive(Print_QueueHandle, Transmit_Symbol, portMAX_DELAY);
		/*	Создание вектора типа uint8_t, состоящего из данных нажатия на кнопку
		 * для передачи через UART
		 */
		for (uint8_t	Number_Bits = 0; Number_Bits < 7; Number_Bits ++)
		{
			switch(Number_Bits)
			{
				case FIRST_BIT:
				{
					Sumbol_Bits[Number_Bits]	=	Transmit_Symbol -> First_Bit + Offset;
				}break;
				case SECOND_BIT:
				{
					Sumbol_Bits[Number_Bits]	=	Transmit_Symbol -> Second_Bit + Offset;
				}break;
				case THIRD_BIT:
				{
					Sumbol_Bits[Number_Bits]	=	Transmit_Symbol -> Third_Bit + Offset;
				}break;
				case FOURTH_BIT:
				{
					Sumbol_Bits[Number_Bits]	=	Transmit_Symbol -> Fourth_Bit + Offset;
				}break;
				case FIFTH_BIT:
				{
					Sumbol_Bits[Number_Bits]	=	Transmit_Symbol -> Fifth_Bit + Offset;
				}break;
				case SIXTH_BIT:
				{
					Sumbol_Bits[Number_Bits]	=	Transmit_Symbol -> Sixth_Bit + Offset;
				}break;
				default:
				{
				};
			}
			if (Sumbol_Bits[Number_Bits] == (uint8_t) 0)
			{
				break;
			}
		}
		Sumbol_Bits[END_BIT] 					=	0;
		xSemaphoreTake(Guardian_UARTHandle, portMAX_DELAY);
		HAL_USART_Transmit_IT(&husart2, Sumbol_Bits, 6);

		/*	Ожидание завершения передачи	*/
		while(HAL_USART_GetState(&husart2) == HAL_USART_STATE_BUSY_TX )
		{
		}

		xSemaphoreGive(Guardian_UARTHandle);

	}
  /* USER CODE END SetUART */
}

/* USER CODE BEGIN Header_GetUART */
/**
* @brief Function implementing the Get_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GetUART */
void GetUART(void *argument)
{
	/* USER CODE BEGIN GetUART */
	uint8_t	*Sumbol_Bits[6];
	/* Infinite loop */
	xSemaphoreTake(Guardian_UARTHandle, portMAX_DELAY);
	for(;;)
	{
		ActivTaskNumbe	=	4;
		xSemaphoreTake(Guardian_UARTHandle, portMAX_DELAY);
		HAL_USART_Receive_IT(&husart2, Sumbol_Bits, 6);

		/*	Ожидание завершения приёма	*/
		while(HAL_USART_GetState(&husart2) == HAL_USART_STATE_BUSY_RX )
		{
		}

		xSemaphoreGive(Guardian_UARTHandle);

		/*	Динамическая индикация, на основе принятых данных	*/
		for (uint8_t	Number_Bits = 0; Number_Bits < 7; Number_Bits ++)
		{
			if (Sumbol_Bits[Number_Bits] - Offset == Point)
			{
				HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET );
				osDelay(TIME_DELAY_SPH);
				HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET );
			}
			else
			{
				HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET );
				osDelay(2 * TIME_DELAY_SPH);
				HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET );
			}
		}

	}
	/* USER CODE END GetUART */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
