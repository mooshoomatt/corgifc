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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "BMI088.h"
#include "PID1.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* PID CONTROLLER */
PID1 PID;

/* GLOBAL FLAGS */
volatile uint8_t DATA_STATUS   = DATA_RESET;   // DATA READY FLAG
volatile uint8_t UPDATE_STATUS = UPDATE_RESET; // UPDATE READY FLAG

/* PID GAINS */
const float Kp = 1.0;
const float Ki = 0.1;
const float Kd = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Overload the _write function so that printf goes to the debug console */
/*
int _write(int file, char *ptr, int len)
{
  for(int i = 0; i < len; i++)
    ITM_SendChar((*ptr++));
  return len;
}
*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* CONSTANTS */
  const float pi = 3.14159265;
  const float GYRO_RATE_SCALE = 2000.0;
  const float RATES[] = {XRATE, YRATE, ZRATE};

  /* ORIENTATION */
  float rot[] = {0.0, 0.0, 0.0};

  /* SETPOINT */
  float set[] = {0.0, 0.0, 0.0};

  /* TIMEKEEPING VARIABLES */
  uint16_t tprev;        // PREVIOUS TIMER VALUE
  uint16_t telapsed;     // ELAPSED CYCLES

  /* BUFFERS */
  char    tx_buf[64];    // TX BUFFER
  uint8_t gyro_buf[6];   // GYROSCOPE BYTE BUFFER

  float  gyro_rate[]  = {0.0, 0.0, 0.0}; // GYROSCOPE RATE BUFFER
  float  stick_rate[] = {0.0, 0.0, 0.0}; // CONTROL RATE BUFFER

  /* TEMPVARS */
  int16_t temp;

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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // CHECK DEVICE IDENTIFIERS
  if ( BMI088_I2C_Read_CHIP_IDS(&hi2c1) != HAL_OK ) { Error_Handler(); }

  // RUN BMI088 GYROSCOPE INITIALIZATION
  if ( BMI088_I2C_GYRO_INIT(&hi2c1) != HAL_OK )     { Error_Handler(); };

  // INITALIZE PID CONTROLLER
  if ( PID1_Init(&PID, Kp, Ki, Kd) != PID_OK )                   { Error_Handler(); }
  if ( PID1_Set_Tau(&PID, 0.02) != PID_OK)                       { Error_Handler(); }
  if ( PID1_Set_Integrator_Limit(&PID, -100.0, 100.0) != PID_OK) { Error_Handler(); }
  if ( PID1_Set_Output_Limit(&PID, -100.0, 100.0) != PID_OK)     { Error_Handler(); }

  // START TIMERS
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start(&htim6);

  // Get starting time
  tprev = __HAL_TIM_GET_COUNTER(&htim6);

  // TURN ON STATUS LED
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // CHECK IF DATA_READY FLAG IS SET
	  if (DATA_STATUS == DATA_READY)
	  {
		  // READ GYROSCOPE
		  if ( BMI088_I2C_Read_Gyro(&hi2c1, gyro_buf) != HAL_OK ) { Error_Handler(); }

		  // UPDATE TIMER
		  telapsed = __HAL_TIM_GET_COUNTER(&htim6) - tprev;
		  tprev    = tprev + telapsed;

		  // CONVERT TO SIGNED INTEGER, SCALE, AND INTEGRATE
		  for (int i = 0; i < 3; i++)
		  {
			  temp         = gyro_buf[2*i + 1] << 8 | gyro_buf[2*i];
			  gyro_rate[i] = ((float)temp*GYRO_RATE_SCALE*pi)/(32767.0*180.0);
			  rot[i]       = rot[i] + 0.000001*(float)telapsed*gyro_rate[i];
		  }
		  PID.meas = rot[0];

		  // UPDATE ROTATION SETPOINT
		  for (int i = 0; i < 3; i++)
		  {
			  // {TODO} GET PWM RAW DATA
			  // {TODO} CONVERT PWM DATA
			  // {TODO} CALCULTE stick_rate[i] as a function of RATES[i]
			  set[i] = set[i] + 0.000001*(float)telapsed*stick_rate[i];
		  }

		  // IMPLEMENT PID ALGORITHM
		  if (PID1_Update(&PID, 0.000001*(float)telapsed) != PID_OK) { Error_Handler(); }
		  for (int i = 0; i < 3; i++)
		  {
			  // {TODO} CALCULATE AXIS ERROR
			  // {TODO} CALCULATE P TERM
			  // {TODO} CALCULATE I TERM
			  // {TODO} CALCULATE D TERM
		  }

		  // UPDATE MOTOR SETTINGS
		  for (int i = 0; i < 3; i++)
		  {
			  // {TODO} UPDATE AXIS PWM RATE
		  }

		  // RESET DATA_READY FLAG
		  DATA_STATUS = DATA_RESET;
	  }

	  // CHECK IF UPDATE_READY FLAG IS SET
	  if (UPDATE_STATUS == UPDATE_READY)
	  {
		  // SEND ORIENTATION DATA OVER VIRTUAL COM PORT
		  // DATA FORMAT: [X ANGLE]    [Y ANGLE]    [Z ANGLE]    [COMPUTATION TIME (uSec)]
		  sprintf(tx_buf, "%f\t%f\t%f\t%f\n", rot[0], rot[1], rot[2], PID.output);
		  CDC_Transmit_FS((uint8_t*)tx_buf, strlen(tx_buf));

		  // RESET UPDATE_READY FLAG
		  UPDATE_STATUS = UPDATE_RESET;
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65536 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 72 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 20000 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : GYRO_INT_Pin */
  GPIO_InitStruct.Pin = GYRO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* GYROSCOPE DATA READY INTERRUPT CALLBACK */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_14)
	{
		// SET DATA_READY FLAG
		DATA_STATUS = DATA_READY;
	}
}

/* TIMER INTERRUPT FOR 50Hz UPDATE OVER SERIAL */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7)
	{
		// SET UPDATE_READY FLAG
		UPDATE_STATUS = UPDATE_READY;
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

  // {TODO} SET ALL MOTOR OUTPUTS TO ZERO!

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  CDC_Transmit_FS((uint8_t*)"ERROR\n", 6);

  while (1)
  {
	  // {TODO} RETRY SENSOR CONNECTION RESTART MAIN() IF SUCCESSFUL
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
