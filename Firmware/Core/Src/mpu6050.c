#include "mpu6050.h"
#include "math.h"

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
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
	HAL_I2C_Init(hi2c);

	// --- ここから本来のWHO_AM_I読み取り処理 ---
	// I2Cアドレス(0x68)の、WHO_AM_Iレジスタ(0x75)を読み取る
	uint8_t who_am_i = 0;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, (0x68 << 1), 0x75, 1, &who_am_i, 1, 100);

	if (ret != HAL_OK || who_am_i != 0x70) {
		return HAL_ERROR;
	}

	// スリープ解除
	uint8_t pwr_mgmt_data = 0x00;
	ret = HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU_REG_PWR_MGMT_1, 1, &pwr_mgmt_data, 1, HAL_MAX_DELAY);

	return ret;
}

// 物理データの読み取り関数
HAL_StatusTypeDef MPU6050_Read_Physical(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *sensor) {
	uint8_t data_buffer[14];

// ACCEL_XOUT_H (0x38)から14バイト連続で呼び出し
	HAL_StatusTypeDef ret =
			HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU_REG_ACCEL_XOUT_H, 1, data_buffer, 14, HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		return ret;
	}

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

	return HAL_OK;
}

// キャブレーション関数
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *sensor) {
	int num_samples = 500;
	float sum_gyro_x = 0.0f;
	float sum_gyro_y = 0.0f;

	for (int i = 0; i < num_samples; i++) {
		MPU6050_Read_Physical(hi2c, sensor); // 関数を呼び出して最新データを取得
		sum_gyro_x += sensor->gyro_x_dps;
		sum_gyro_y += sensor->gyro_y_dps;
		HAL_Delay(2); // 2ms待機 (全体で約1秒間の計測)
	}

// 平均値をオフセット（初期ズレ）として保存
	sensor->gyro_x_offset = sum_gyro_x / num_samples;
	sensor->gyro_y_offset = sum_gyro_y / num_samples;

// 現在の重力方向を読み取る
	MPU6050_Read_Physical(hi2c, sensor);
	float initial_roll = atan2f(sensor->accel_y_g, sensor->accel_z_g) * (180.0f / PI);
	float initial_pitch = atan2f(-sensor->accel_x_g, sqrtf(sensor->accel_y_g * sensor->accel_y_g
			+ sensor->accel_z_g * sensor->accel_z_g)) * (180.0f / PI);

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
	sensor->acc_only_roll = atan2f(sensor->accel_y_g, sensor->accel_z_g) * (180.0f / PI);
	sensor->gyro_only_roll += sensor->gyro_x_dps * DT;
	sensor->comp_roll = ALPHA * (sensor->comp_roll + sensor->gyro_x_dps * DT) + (1.0f - ALPHA) * sensor->acc_only_roll;

// ピッチ角の計算
	sensor->acc_only_pitch = atan2f(-sensor->accel_x_g, sqrtf(sensor->accel_y_g * sensor->accel_y_g
			+ sensor->accel_z_g * sensor->accel_z_g)) * (180.0f / PI);
	sensor->gyro_only_pitch += sensor->gyro_y_dps * DT;
	sensor->comp_pitch = ALPHA * (sensor->comp_pitch + sensor->gyro_y_dps * DT)
			+ (1.0f - ALPHA) * sensor->acc_only_pitch;
}

