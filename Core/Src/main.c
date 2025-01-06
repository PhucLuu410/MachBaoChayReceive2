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
#include "string.h"
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
int a = 0;
char SIM800__Call_Char[] = "ATD0765214176;\r\n";
char SIM800__Mess_Char1[] = "AT+CMGF=1;\r\n";
char SIM800__Mess_Char2[] = "AT+CMGS=\"0765214176\"\r\n";
char SIM800__Mess_Char3[] = "CANH BAO SU CO;\r\n";
char SIM800__Mess_Char4[] = "\x1A\r\n";
uint32_t ThoiGianCuocGoiCuoi = 0;
uint32_t ThoiGianTinNhanCuoi = 0;
uint32_t ThoiGianTinHieuCuoi = 0;
uint8_t CoTinNhan, CoCuocGoi, CoKiemTraHeThong;
uint8_t DataNhanTuCamera;
uint32_t last_time;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TatHeThongChuaChay(void);
void BatHeThongChuaChay(void);
void BatHeThongCanhBao(void);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_UART_Receive(&huart1, &DataNhanTuCamera, 1, 100);
		if (((HAL_GPIO_ReadPin(TINHIEUNHIETDO_GPIO_Port, TINHIEUNHIETDO_Pin) == 0)
			  && (HAL_GPIO_ReadPin(TINHIEULUA_GPIO_Port, TINHIEULUA_Pin) == 0)
		      && (HAL_GPIO_ReadPin(TINHIEUKHOI_GPIO_Port, TINHIEUKHOI_Pin) == 0)
			  && (HAL_GPIO_ReadPin(TINHIEUKHIGAS_GPIO_Port, TINHIEUKHIGAS_Pin) == 0))
			  || DataNhanTuCamera != '0' )
		{
			if(CoKiemTraHeThong == 0)
			{
				ThoiGianTinHieuCuoi = HAL_GetTick();
				CoKiemTraHeThong = 1;
			}
			if(HAL_GetTick() - ThoiGianTinHieuCuoi >= 5000)
			{
				TatHeThongChuaChay();
				CoTinNhan = 0;
				CoCuocGoi = 0;
				CoKiemTraHeThong = 0;
				ThoiGianTinHieuCuoi = 0;
			}
		}


		if ((HAL_GPIO_ReadPin(TINHIEUNHIETDO_GPIO_Port, TINHIEUNHIETDO_Pin) == 1
			|| HAL_GPIO_ReadPin(TINHIEUKHOI_GPIO_Port, TINHIEUKHOI_Pin) == 1
			|| HAL_GPIO_ReadPin(TINHIEUKHIGAS_GPIO_Port, TINHIEUKHIGAS_Pin) == 1))
		{
			BatHeThongCanhBao();
		    if (CoTinNhan == 0)
		    {
		        HAL_UART_Transmit(&huart2, (uint8_t*)SIM800__Mess_Char1, strlen(SIM800__Mess_Char1), HAL_MAX_DELAY);
		        HAL_Delay(500);
		        HAL_UART_Transmit(&huart2, (uint8_t*)SIM800__Mess_Char2, strlen(SIM800__Mess_Char2), HAL_MAX_DELAY);
		        HAL_Delay(500);
		        HAL_UART_Transmit(&huart2, (uint8_t*)SIM800__Mess_Char3, strlen(SIM800__Mess_Char3), HAL_MAX_DELAY);
		        HAL_Delay(500);
		        HAL_UART_Transmit(&huart2, (uint8_t*)SIM800__Mess_Char4, strlen(SIM800__Mess_Char4), HAL_MAX_DELAY);
		        HAL_Delay(2000);
		        CoTinNhan = 1;
		    }
		}
		else if (HAL_GPIO_ReadPin(TINHIEULUA_GPIO_Port, TINHIEULUA_Pin) == 1 || DataNhanTuCamera == '1')
		{
	    	BatHeThongChuaChay();
		    if (CoCuocGoi == 0)
		    {
		        if (HAL_GetTick() - ThoiGianCuocGoiCuoi >= 5000)
		        {
		            HAL_UART_Transmit(&huart2, (uint8_t*)SIM800__Call_Char, strlen(SIM800__Call_Char), HAL_MAX_DELAY);
		            ThoiGianCuocGoiCuoi = HAL_GetTick();
		        }
		    }
		}
		else
		{
		    CoTinNhan = 0;
		    CoCuocGoi = 0;
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  huart3.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TINHIEUGUI_Pin|COICHIP_Pin|RELAY1_Pin|RELAY2_Pin
                          |OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TINHIEUNHIETDO_Pin TINHIEUKHOI_Pin TINHIEUKHIGAS_Pin TINHIEULUA_Pin */
  GPIO_InitStruct.Pin = TINHIEUNHIETDO_Pin|TINHIEUKHOI_Pin|TINHIEUKHIGAS_Pin|TINHIEULUA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TINHIEUGUI_Pin COICHIP_Pin RELAY1_Pin RELAY2_Pin
                           OUT1_Pin */
  GPIO_InitStruct.Pin = TINHIEUGUI_Pin|COICHIP_Pin|RELAY1_Pin|RELAY2_Pin
                          |OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TINHIEUKHIGAS2_Pin TINHIEUKHOI2_Pin TINHIEUNHIETDO2_Pin */
  GPIO_InitStruct.Pin = TINHIEUKHIGAS2_Pin|TINHIEUKHOI2_Pin|TINHIEUNHIETDO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TatHeThongChuaChay(void)
{
	HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, 0);
	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, 0);
	HAL_GPIO_WritePin(COICHIP_GPIO_Port, COICHIP_Pin, 0);
}


void BatHeThongChuaChay(void)
{
	HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, 1);
	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, 1);
	HAL_GPIO_WritePin(COICHIP_GPIO_Port, COICHIP_Pin, 1);
}


void BatHeThongCanhBao(void)
{
	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, 1);
	HAL_GPIO_WritePin(COICHIP_GPIO_Port, COICHIP_Pin, 1);
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
