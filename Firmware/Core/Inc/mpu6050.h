// Core/Inc/mpu6050.h
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

// MPU-6050のI2Cアドレス
#define MPU6050_ADDR 0xD0

// レジスタアドレス
#define MPU_REG_PWR_MGMT_1 0x68
#define MPU_REG_ACCEL_XOUT_H 0x3B

#define PI 3.14159265358979f
#define DT 0.01f
#define ALPHA 0.99f

// センターデータ構造
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

// --- 関数 ---
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read_Physical(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *sensor);
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *sensor);
void MPU6050_Calculate_Attitude(MPU6050_Data_t *sensor);

#endif /* INC_MPU6050_H_ */
