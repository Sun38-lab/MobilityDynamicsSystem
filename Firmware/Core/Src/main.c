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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// MPU-6050のI2Cアドレス
#define MPU6050_ADDR 0xD0

//レジスタアドレス
#define MPU_REG_PWR_MGMT_1 0x68
#define MPU_REG_ACCEL_XOUT_H 0x3B

#define PI 3.14159265358979f
#define DT 0.01f

//フィルター関数（ジャイロを99%信用、加速度で1%補正）
#define ALPHA 0.99f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct {
	// 生データ
	int16_t raw_accel_x, raw_accel_y, raw_accel_z;
	int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;

	// 物理量（G, deg/s）格納用
	float accel_x_g, accel_y_g, accel_z_g;
	float gyro_x_dps, gyro_y_dps, gyro_z_dps;

	//　誤差記憶用
	float gyro_x_offset;
	float gyro_y_offset;
	float gyro_z_offset;

	//　ロール角用
	float acc_only_roll;
	float gyro_only_roll; // ジャイロの角度蓄積用
	float comp_roll; //　補正フィルター後の角度

	//　ピッチ角用
	float acc_only_pitch;
	float gyro_only_pitch;
	float comp_pitch;
} MPU6050_Data_t;

//　UART送信用バッファ
char uart_buf[100];
volatile uint8_t timer_10ms_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Read_Physical(MPU6050_Data_t *sensor);
void MPU6050_Calibrate(MPU6050_Data_t *sensor);
void MPU6050_Calculate_Attitude(MPU6050_Data_t *sensor);

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

	sprintf(uart_buf, "\r\n--- Executing I2C Bus Recovery ---\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 100);

	// 1. PB8(SCL)とPB9(SDA)を一時的に「手動スイッチ(GPIO出力)」に変更
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 100);

	// I2Cアドレス(0x68)の、WHO_AM_Iレジスタ(0x75)を読み取る
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x75, 1, &who_am_i, 1, 100);

	if (ret == HAL_OK) {
		sprintf(uart_buf, "Success! WHO_AM_I = 0x%02X\r\n", who_am_i);
	} else {
		sprintf(uart_buf, "Failed. Error Code = %d\r\n", ret);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 100);

	uint8_t pwr_mgmt_data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_REG_PWR_MGMT_1, 1, &pwr_mgmt_data, 1, HAL_MAX_DELAY);

	// 初期化完了のメッセージをTeraTermに送信(sprintfで文字を上書き、Taransmitで送信)
	sprintf(uart_buf, "MPU-6050 Wake Up Complete!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
	HAL_Delay(100);

	// キャリブレーションの実行（アドレスを渡す）
	sprintf(uart_buf, "Calibration Gyro... Please keep it still.\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

	MPU6050_Calibrate(&sensor_data);

	sprintf(uart_buf, "Calibration Done! Offset X:%.2f, Y:%.2f\r\n", sensor_data.gyro_x_offset, sensor_data.gyro_y_offset);
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

	HAL_TIM_Base_Start_IT(&htim3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		// 10msタイマー割り込みが発生したかチェック
		if (timer_10ms_flag == 1) {
			timer_10ms_flag = 0; // フラグを下ろす
			// データの取得と補正
			MPU6050_Read_Physical(&sensor_data);
			MPU6050_Calculate_Attitude(&sensor_data);

			// UART送信
			sprintf(uart_buf, "$MPU,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
								sensor_data.accel_x_g, sensor_data.accel_y_g, sensor_data.accel_z_g,
								sensor_data.gyro_x_dps, sensor_data.gyro_y_dps, sensor_data.gyro_z_dps,
								sensor_data.acc_only_roll, sensor_data.gyro_only_roll,
								sensor_data.comp_roll, sensor_data.comp_pitch);
			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
		}
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
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
// 物理データの読み取り関数
void MPU6050_Read_Physical(MPU6050_Data_t *sensor) {
	uint8_t data_buffer[14];

	// ACCEL_XOUT_H (0x38)から14バイト連続で呼び出し
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_REG_ACCEL_XOUT_H, 1, data_buffer, 14, HAL_MAX_DELAY);

	//生データへ変換
	sensor->raw_accel_x = (int16_t) (data_buffer[0] << 8 | data_buffer[1]);
	sensor->raw_accel_y = (int16_t) (data_buffer[2] << 8 | data_buffer[3]);
	sensor->raw_accel_z = (int16_t) (data_buffer[4] << 8 | data_buffer[5]);
	sensor->raw_gyro_x = (int16_t) (data_buffer[8] << 8 | data_buffer[9]);
	sensor->raw_gyro_y = (int16_t) (data_buffer[10] << 8 | data_buffer[11]);
	sensor->raw_gyro_z = (int16_t) (data_buffer[12] << 8 | data_buffer[13]);

	sensor->accel_x_g = (float) sensor->raw_accel_x / 16384.0f;
	sensor->accel_y_g = (float) sensor->raw_accel_y / 16384.0f;
	sensor->accel_z_g = (float) sensor->raw_accel_z / 16384.0f;
	sensor->gyro_x_dps = (float) sensor->raw_gyro_x / 131.0f;
	sensor->gyro_y_dps = (float) sensor->raw_gyro_y / 131.0f;
	sensor->gyro_z_dps = (float) sensor->raw_gyro_z / 131.0f;
}

// キャブレーション関数
void MPU6050_Calibrate(MPU6050_Data_t *sensor) {
	int num_samples = 500;
	float sum_gyro_x = 0.0f;
	float sum_gyro_y = 0.0f;

	for (int i = 0; i < num_samples; i++) {
		MPU6050_Read_Physical(sensor); // 関数を呼び出して最新データを取得
		sum_gyro_x += sensor->gyro_x_dps;
		sum_gyro_y += sensor->gyro_y_dps;
		HAL_Delay(2); // 2ms待機 (全体で約1秒間の計測)
	}

	// 平均値をオフセット（初期ズレ）として保存
	sensor->gyro_x_offset = sum_gyro_x / num_samples;
	sensor->gyro_y_offset = sum_gyro_y / num_samples;

	// 現在の重力方向を読み取る
	MPU6050_Read_Physical(sensor);
	float initial_roll = atan2f(sensor->accel_y_g, sensor->accel_z_g) * (180.0f / PI);
	float initial_pitch = atan2f(-sensor->accel_x_g, sqrtf(sensor->accel_y_g * sensor->accel_y_g + sensor->accel_z_g * sensor->accel_z_g)) * (180.0f / PI);

	// 変数のスタート地点を「現在の本当の角度」に合わせる
	sensor->gyro_only_roll = initial_roll;
	sensor->comp_roll = initial_roll;
	sensor->gyro_only_pitch = initial_pitch;
	sensor->comp_pitch = initial_pitch;
}

// 補正フィルター関数
void MPU6050_Calculate_Attitude(MPU6050_Data_t *sensor) {
	//オフセットを引く
	sensor->gyro_x_dps -= sensor->gyro_x_offset;
	sensor->gyro_y_dps -= sensor->gyro_y_offset;

	// ロール角の計算
	float acc_only_roll = atan2f(sensor->accel_y_g, sensor->accel_z_g) * (180.0f / PI);
	sensor->gyro_only_roll += sensor->gyro_x_dps * DT;
	sensor->comp_roll = ALPHA * (sensor->comp_roll + sensor->gyro_x_dps * DT) + (1.0f - ALPHA) * acc_only_roll;

	// ピッチ角の計算
	float acc_only_pitch = atan2f(-sensor->accel_x_g, sqrtf(sensor->accel_y_g * sensor->accel_y_g
			+ sensor->accel_z_g * sensor->accel_z_g)) * (180.0f / PI);
	sensor->gyro_only_pitch += sensor->gyro_y_dps * DT;
	sensor->comp_pitch = ALPHA * (sensor->comp_pitch + sensor->gyro_y_dps * DT) + (1.0f - ALPHA) * acc_only_pitch;
}

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
