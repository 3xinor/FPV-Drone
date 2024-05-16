/*
 * imu.c
 *
 *  Created on: May 15, 2024
 *      Author: David Exinor
 */


#include "imu.h"

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

/* Static Functions */

static void read_accel(accel* drone_accel) {
	uint8_t rec_data[6];
	// read 6 bytes of data starting from ACCEL_XOUT_H Register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6, 1000);

	Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
	Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
	Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);

	// convert raw values into g's and store in accel struct
	drone_accel->x = Accel_X_RAW / 16384.0;
	drone_accel->y = Accel_Y_RAW / 16384.0;
	drone_accel->z = Accel_Z_RAW / 16384.0;
}

static void read_gyro(gyro* drone_gyro) {
	uint8_t rec_data[6];
	// read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data, 6, 1000);

	Gyro_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
	Gyro_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
	Gyro_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);

	// convert raw values into degrees per second (dps) and store in gyro struct
	drone_gyro->x = Gyro_X_RAW / 131.0;
	drone_gyro->y = Gyro_Y_RAW / 131.0;
	drone_gyro->z = Gyro_Z_RAW / 131.0;
}

/* Functions */
bool init_mpu6050() {
	// Check if sensor is responding
	uint8_t check;
	uint8_t data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if (check == 0x68) {
		// wake up sensor using power management register
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MNGMT_1_REG, 1, &data, 1, 1000);

		// set data rate of 1Khz by writing to SMPLRT_DIV register
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 2g
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

		// Set Gyroscopes configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> 250 deg/s
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

		return true;
	}

	return false;
}

void read_mpu6050(mpu* drone_mpu) {
	read_accel(drone_mpu->mpu_accel);
	read_gyro(drone_mpu->mpu_gyro);
}
