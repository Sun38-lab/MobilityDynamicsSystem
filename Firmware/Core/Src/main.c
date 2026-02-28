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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "mpu6050.h"
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
typedef enum {
	STATE_INIT,			//　初期化
	STATE_CALIBRATION,	//　センサーのキャリブレーション
	STATE_NORMAL,		// 通常動作
	STATE_ERROR			//　エラー状態
} SystemState_t;

// 現在の状態を保持
SystemState_t current_state = STATE_INIT;
char uart_buf[100]; //　UART送信用バッファ
volatile uint8_t timer_10ms_flag;

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
int main(void) {

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
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	MPU6050_Data_t sensor_data = { 0 };
	char uart_buf[70];
	uint8_t who_am_i = 0;
	uint8_t pwr_mgmt_data = 0x00;
	HAL_StatusTypeDef status;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		switch (current_state) {

		case STATE_INIT:
			// ==========================================
			// 状態1：初期化 (I2C復旧、MPU6050の起動)
			// ==========================================

			sprintf(uart_buf, "\r\n--- Executing I2C Bus Recovery ---\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 100);

			if (MPU6050_Init(&hi2c1) == HAL_OK) {
				current_state = STATE_CALIBRATION;
			} else {
				sprintf(uart_buf, "MPU-6050 Knit Failed! Moving to Error State.\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 100);

				// 異常時はエラー状態へ
				current_state = STATE_ERROR;
			}
			break;

		case STATE_CALIBRATION:
			// ==========================================
			// 状態2：キャブレーション
			// ==========================================

			// キャリブレーションの実行（アドレスを渡す）
			sprintf(uart_buf, "Calibration Gyro... Please keep it still.\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

			MPU6050_Calibrate(&hi2c1, &sensor_data);

			sprintf(uart_buf, "Calibration Done! Offset X:%.2f, Y:%.2f\r\n", sensor_data.gyro_x_offset, sensor_data.gyro_y_offset);
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 100);

			// タイマー割込み開始
			HAL_TIM_Base_Start_IT(&htim3);

			// 通常動作状態へ遷移
			current_state = STATE_NORMAL;

			break;

		case STATE_NORMAL:
			// ==========================================
			// 状態3：通常動作 (100Hzでデータ取得・送信)
			// ==========================================

			// 10msタイマー割り込みが発生したかチェック
			if (timer_10ms_flag == 1) {
				timer_10ms_flag = 0; // フラグを下ろす

				if (MPU6050_Read_Physical(&hi2c1, &sensor_data) == HAL_OK) {
					MPU6050_Calculate_Attitude(&sensor_data);
					sprintf(uart_buf, "$MPU,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", sensor_data.accel_x_g, sensor_data.accel_y_g, sensor_data.accel_z_g, sensor_data.gyro_x_dps, sensor_data.gyro_y_dps, sensor_data.gyro_z_dps, sensor_data.acc_only_roll, sensor_data.gyro_only_roll, sensor_data.comp_roll, sensor_data.comp_pitch);
					HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
				} else {
					// I2C通信で委譲が発生した場合
					HAL_TIM_Base_Stop_IT(&htim3);
					current_state = STATE_ERROR;
				}
			}
			break;

		case STATE_ERROR:
			// ==========================================
			// 状態4：エラー・フェールセーフ状態
			// ==========================================

			// 安全のための処理（例：モーターのPWM出力を強制0にする等）
			sprintf(uart_buf, "SYSTEM ERROR: Sensor Disconnected or I2C Fault.\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 100);

			// 1秒待機して再初期化を試みる、もしくは無限ループでシステムを安全に停止させる
			HAL_Delay(1000);

			// current_state = STATE_INIT;
			break;

		default:
			current_state = STATE_ERROR;
			break;
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */

			/* USER CODE END 3 */

		}
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

// タイマー割り込みが発生したときに呼ばれるコールバック関数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// TIM3の割り込みかどうかを確認
	if (htim->Instance == TIM3) {
		timer_10ms_flag = 1; // 10ms経過フラグを立てる
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
