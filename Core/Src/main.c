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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "math.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int BUFF_SIZE = 14;
int Pulse;

/* Tính toán timer
Fs = (APB1 Timer Clocks) / (PSC + 1) = ... (KHz)
Ts = (1/Fs) * (Counter Period + 1) = ... (ms)
*/

int16_t counter = 0;
int16_t pre_counter = 0;

uint8_t STX[] = {0x02};
uint8_t ETX[] = {0x03};
uint8_t ACK[] = {0x06};

uint8_t PULSE[] = {0x50, 0x55, 0x4C, 0x53, 0x45};

uint8_t PICK[] = {0x00, 0x00, 0x00, 0x04};
uint8_t PLACE[] = {0x00, 0x00, 0x00, 0x05};

uint8_t RxData[14];
uint8_t TxData[14];
uint8_t Index = 0;
uint8_t Data;

uint8_t subData[8];
uint8_t strCommand[5];
uint8_t strOpt[4];
uint8_t strData[2];

bool flag_DataAvailable;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t *subString(uint8_t *s, int pos, int index)
{
	memset(subData,0,sizeof(subData));
	int j = pos;
	int k = pos + index;
	for (int i = pos; i < k; i++)
	{
		subData[i-j] = s[i];
	}
	return subData;
}

bool ReadComm(uint8_t *pBuff, uint8_t nSize)
{
	if ((pBuff[0] == STX[0]) && (pBuff[13] == ETX[0]))
	{
		memcpy(strCommand, subString(pBuff, 1, 5), 5);
		memcpy(strOpt, subString(pBuff, 6, 4), 4);
		memcpy(strData, subString(pBuff, 10, 2), 2);

		flag_DataAvailable = true;
	}
	else
	{
		flag_DataAvailable = false;
	}
	return flag_DataAvailable;
}

bool WriteComm(uint8_t *pBuff, uint8_t nSize)
{
	return HAL_UART_Transmit(&huart2, pBuff, nSize, 1000);
}
void Datasend(void)
{
	Index = 0;
	memcpy(TxData + Index, STX, 1);
	Index +=1;
	memcpy(TxData + Index, (uint8_t *)strCommand, 5);
	Index +=5;
	memcpy(TxData + Index, (uint8_t *)strOpt, 4);
	Index +=4;
	memcpy(TxData + Index, (uint8_t *)strData, 2);
	Index +=2;
	memcpy(TxData + Index, ACK, 1);
	Index +=1;
	memcpy(TxData + Index, ETX, 1);

	WriteComm(TxData, BUFF_SIZE);
}
void Encoder(void)
{
	counter = htim2.Instance->CNT - pre_counter;
	pre_counter = htim2.Instance->CNT;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim5.Instance)
		Encoder();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		ReadComm(RxData, BUFF_SIZE);
		HAL_UART_Receive_IT(&huart2, (uint8_t *)RxData, BUFF_SIZE);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t *)RxData, BUFF_SIZE);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (flag_DataAvailable == true)
		{
			Pulse = atoi((char *)strData);
			
			//Duty (%) = (CCR / ARR) * 100 (ARR = const = 1000)
			//htim1.Instance->CCR1 = Pulse;
			htim1.Instance->CCR1 = Pulse;
			Datasend();
			flag_DataAvailable = false;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
