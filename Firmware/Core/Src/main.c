/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  char uart_buf[50];
//  sprintf(uart_buf,"Starting I2C Scan...\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//  //1から127までのI2Cアドレスをスキャン
//  for(uint8_t i = 1; i < 128; i++){
//	  //STM32のHALライブラリでは、I2Cアドレスを左に1ビットシフトして渡す使用
//	  HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2,2);
//
//	  //応答があった場合
//	  // 応答があった場合（HAL_OK）
//	      if (result == HAL_OK) {
//	          sprintf(uart_buf, "I2C device found at address 0x%02X\r\n", i);
//	          HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//	      }
//  }
//
//sprintf(uart_buf,"I2C Scam Complete.\r\n");
//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);


//  char uart_buf[50];
//  uint8_t who_am_i = 0;
//
//  sprintf(uart_buf, "Checking MPU-6050 WHO_AM_I...\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//  // STM32内部のI2C回路を強制リセットして叩き起こす
//  __HAL_RCC_I2C1_FORCE_RESET();
//  HAL_Delay(10);
//  __HAL_RCC_I2C1_RELEASE_RESET();
//  HAL_I2C_Init(&hi2c1); // I2Cを再初期化
//  HAL_Delay(10);
//
//  // I2Cアドレス(0x68)の、WHO_AM_Iレジスタ(0x75)を1バイト直接読み取る
//  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x75, 1, &who_am_i, 1, 100);
//
//  if (ret == HAL_OK) {
//      // 成功した場合
//      sprintf(uart_buf, "Success! WHO_AM_I = 0x%02X\r\n", who_am_i);
//  } else {
//      // 失敗した場合
//      sprintf(uart_buf, "Failed. Error Code = %d\r\n", ret);
//  }
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);


//  char uart_buf[50];
//
//  sprintf(uart_buf, "\r\n--- I2C Hardware Diagnosis ---\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//  // PB8(SCL)とPB9(SDA)の「実際の電圧（High=1, Low=0）」を直接読み取る
//  GPIO_PinState scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
//  GPIO_PinState sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
//
//  // 結果をTeraTermに送信
//  sprintf(uart_buf, "SCL (PB8) Level = %d\r\n", scl);
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//  sprintf(uart_buf, "SDA (PB9) Level = %d\r\n", sda);
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);
//
//  if (scl == 1 && sda == 1) {
//      sprintf(uart_buf, "Result: Hardware OK (1,1). STM32 I2C is frozen!\r\n");
//  } else {
//      sprintf(uart_buf, "Result: Hardware Error. Line is pulled LOW.\r\n");
//  }
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);



  char uart_buf[70];
  uint8_t who_am_i = 0;

  sprintf(uart_buf, "\r\n--- Executing I2C Bus Recovery ---\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);

  // 1. PB8(SCL)とPB9(SDA)を一時的に「手動スイッチ(GPIO出力)」に変更
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // オープンドレイン出力
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 2. 通信終了(STOPコンディション)の波形を強制的に手動で作り出す
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL High
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // SDA High
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // SDA Low
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL Low
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL High
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // SDA High
  HAL_Delay(1);

  // 3. PB8とPB9を再び「I2C通信ピン」に戻す (STM32F401のI2C1はAF4)
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 4. I2C回路のハードウェアリセットと再初期化
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(2);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_I2C_Init(&hi2c1);

  // --- ここから本来のWHO_AM_I読み取り処理 ---
  sprintf(uart_buf, "Checking MPU-6050 WHO_AM_I...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);

  // I2Cアドレス(0x68)の、WHO_AM_Iレジスタ(0x75)を読み取る
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x75, 1, &who_am_i, 1, 100);

  if (ret == HAL_OK) {
      sprintf(uart_buf, "Success! WHO_AM_I = 0x%02X\r\n", who_am_i);
  } else {
      sprintf(uart_buf, "Failed. Error Code = %d\r\n", ret);
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // PA5の出力を反転
	  HAL_Delay(500);                        // 500ミリ秒待機
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
