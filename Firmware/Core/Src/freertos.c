/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "i2c.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include  "tim.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	float pitch;
	float error;
	float speed;
	float gyro_y;
	int pulse_width;
	int current_pwm;
} TelemetryData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// main.cのグローバル変数を参照する
extern volatile float Kp;
extern volatile float Ki;
extern volatile float Kd;
extern volatile float target_pitch;
extern MPU6050_Data_t global_sensor_data;
/* USER CODE END Variables */
/* Definitions for SensorTaskHandl */
osThreadId_t SensorTaskHandlHandle;
const osThreadAttr_t SensorTaskHandl_attributes = { .name = "SensorTaskHandl", .stack_size = 512 * 4, .priority =
		(osPriority_t) osPriorityHigh, };
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = { .name = "UartTask", .stack_size = 512 * 4, .priority =
		(osPriority_t) osPriorityNormal, };
/* Definitions for SensorDataQueue */
osMessageQueueId_t SensorDataQueueHandle;
const osMessageQueueAttr_t SensorDataQueue_attributes = { .name = "SensorDataQueue" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static float clip_float(float val, float min, float max);
static float caluculate_control_pwm(float target, float current_pitch, float current_velocity, float prev_pwm);
static float caluculate_control_pwm_velocity(float target, float current_pitch, float current_velocity, float current_pwm);

//static float CalculateControlSpeed(float error, float derivative) {
//	float speed = 0.0f;
//	float error_abs = (error > 0.0f) ? error : -error;
//
//	if (error_abs > 10.0f) {
//		// 1.大ズレ：バンバン制御
//		speed = (error > 0) ? 3000.0f : -3000.0f;
//	} else if (error_abs > 4.0f) {
//		// 2.接近フェーズ（4度〜10度）
//		// 摩擦の影響が少ない領域なので、やや強めのKpで一気に寄せる
//		speed = (Kp * error) + (Kd* derivative);
//	} else if (error_abs > 2.0f) {
//		// 3.最終着地フェーズ（例: 2.0度〜10.0度）
//		// 不感帯(2.0度)を「はみ出した分」だけを抽出する
//		float active_error = (error > 0.0f) ? (error - 2.0f) : (error + 2.0f);
//		speed = (Kp * active_error) + (Kd * derivative);
//	} else {
//		// ジンバルが動いている時（ノイズ以上の角速度がある時）はブレーキ(Kd)を優先
//		if (derivative > 5.0f || derivative < -5.0f) {
//			speed = Kd * derivative;
//		} else {
//			speed = Ki * error;
//		}
//	}
//	return speed;
//}

/* USER CODE END FunctionPrototypes */

void StartSensorTask(void *argument);
void StartUartTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of SensorDataQueue */
	SensorDataQueueHandle = osMessageQueueNew(16, 128, &SensorDataQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of SensorTaskHandl */
	SensorTaskHandlHandle = osThreadNew(StartSensorTask, NULL, &SensorTaskHandl_attributes);

	/* creation of UartTask */
	UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument) {
	/* USER CODE BEGIN StartSensorTask */
	/* Infinite loop */
	// {0}で空にするのではなく、キャリブレーション済みのデータを丸ごとコピーして引き継ぐ！
	MPU6050_Data_t local_sensor_data = global_sensor_data;
	uint32_t tick_delay = 10;
	uint32_t tick_update = osKernelGetTickCount();
	TelemetryData_t tx_data;

	for (;;) {

		if (current_state == STATE_NORMAL) {
			if (MPU6050_Read_Physical(&hi2c1, &local_sensor_data) == HAL_OK) {
				MPU6050_Calculate_Attitude(&local_sensor_data);

				// 偏差（エラー）の計算
				float error = target_pitch - local_sensor_data.comp_pitch;
				float derivative = local_sensor_data.gyro_y_dps;

				// StartSensorTask のループ外で定義
				static float current_pwm = 1520.0f; // 前回のPWMを保持する変数
				current_pwm =
						caluculate_control_pwm_velocity(target_pitch, local_sensor_data.comp_pitch, derivative, current_pwm);

//				// 静的変数（メモリ）として現在のPWMを保持し、変化量を足し込み続ける
//				static float current_pwm = 1520.0f;
//				current_pwm += speed * 0.01f; // DT(0.01秒)を掛けて足し込む
				float speed = 0.0f;
//				// サーボの可動範囲(500μs~2400μs)を超えないようにリミット
//				if (current_pwm > 2400.0f)
//					current_pwm = 2400.0f;
//				if (current_pwm < 600.0f)
//					current_pwm = 600.0f;

				int pulse_width = (int) current_pwm;

				// --- フェールセーフ処理：NORMALの時のみPWMを更新 ---
				if (current_state == STATE_NORMAL) {
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width);
				}

				tx_data.current_pwm = current_pwm;
				tx_data.pulse_width = pulse_width;
				tx_data.error = error;
				tx_data.speed = speed;
				tx_data.gyro_y = local_sensor_data.gyro_y_dps;
				tx_data.pitch = local_sensor_data.comp_pitch;

				// 取得したデータをQueueに送信
				osMessageQueuePut(SensorDataQueueHandle, &tx_data, 0, 0);
			} else {
				current_state = STATE_ERROR;
			}
		} else {
			// エラー時のフェールセーフ処理
			current_state = STATE_ERROR; // 状態をエラーに変更
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); //PWM出力停止
		}
		// 次の10ms周期までタスクを正確にブロックする
		tick_update += tick_delay;
		osDelayUntil(tick_update);
	}
	/* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument) {
	/* USER CODE BEGIN StartUartTask */
	/* Infinite loop */
	/* USER CODE BEGIN StartUartTask */
	char uart_buf[120];
	TelemetryData_t rx_data;

	for (;;) {
		// Queueにデータが来るまで無限に待機 (CPUを休ませる)
		if (osMessageQueueGet(SensorDataQueueHandle, &rx_data, NULL, osWaitForever) == osOK) {

			// UART送信
//			sprintf(uart_buf, "$MPU,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", received_data.accel_x_g, received_data.accel_y_g, received_data.accel_z_g, received_data.gyro_x_dps, received_data.gyro_y_dps, received_data.gyro_z_dps, received_data.acc_only_roll, received_data.gyro_only_roll, received_data.comp_roll, received_data.comp_pitch);

			// Debug:UART送信
			sprintf(uart_buf, "DEBUG,Kp:%.2f,Kd:%.2f,Pitch:%.2f,Err:%.2f,Gyro:%.2f,Spd:%.2f,PWM:%d\r\n", Kp, Kd, rx_data.pitch, rx_data.error, rx_data.gyro_y, rx_data.speed, rx_data.pulse_width);

			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, strlen(uart_buf), 20);
		}
	}
	/* USER CODE END StartUartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static inline float clip_float(float val, float min, float max) {
	val = ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)));
	return val;
}

static float caluculate_control_pwm_velocity(float target, float current_pitch, float current_velocity, float current_pwm) {
	float error = target - current_pitch;

	// =======================================================
	// ★ ロバスト性向上の鍵：「未来の誤差」を予測する
	// =======================================================
	// 速度(deg/s) × 想定される遅延時間(s) = 遅延の間に進んでしまう角度
	// ※ Pythonのシミュレータで設定した「dead_time_steps」に相当する時間を入れます。
	// 例: 50ms (0.05s) の遅延を見込む場合
	const float PREDICT_TIME = 0.02f;

	// 現在の速度を加味した「少し先の未来のエラー」
	float future_error = error - (current_velocity * PREDICT_TIME);
	float future_error_abs = (future_error > 0.0f) ? future_error : -future_error;

	float speed = 0.0f;

	// =======================================================
	// ゾーン判定には「未来の誤差（future_error_abs）」を使う！
	// =======================================================
	if (future_error_abs > 10.0f) {
		// 1. 大ズレ：バンバン制御
		speed = (error > 0.0f) ? 3000.0f : -3000.0f;

	} else if (future_error_abs > 4.0f) {
		// 2. 接近フェーズ
		// ※ 出力計算自体には、実際の誤差(error)を使います
		speed = (Kp * error) + (Kd * current_velocity);

	} else if (future_error_abs > 2.0f) {
		// 3. 最終着地フェーズ
		float active_error = (error > 0.0f) ? (error - 2.0f) : (error + 2.0f);
		speed = (Kp * active_error) + (Kd * current_velocity);

	} else {
		// 4. 不感帯
		if (current_velocity > 5.0f || current_velocity < -5.0f) {
			speed = Kd * current_velocity;
		} else {
			speed = Ki * error;
		}
	}

	// 最終スピードリミット
	speed = clip_float(speed, -4000.0f, 4000.0f);

	// PWMの足し込み（45度でも負けない速度型の強み！
	float new_pwm = current_pwm + (speed * DT);

	return clip_float(new_pwm, 600.0f, 2400.0f);
}
/* USER CODE END Application */

