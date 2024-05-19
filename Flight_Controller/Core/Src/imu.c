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

int16_t raw_temp = 0;

float gyro_error_x = 0;
float gyro_error_y = 0;
float gyro_error_z = 0;

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
	drone_gyro->x = (Gyro_X_RAW / 131.0) - gyro_error_x;
	drone_gyro->y = (Gyro_Y_RAW / 131.0) - gyro_error_y;
	drone_gyro->z = (Gyro_Z_RAW / 131.0) - gyro_error_z;
}

static void read_temp(mpu* drone_mpu) {
	uint8_t rec_data[2];
	// read 6 BYTES of data starting from
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1, rec_data, 2, 1000);
	raw_temp = (int16_t)(rec_data[0] << 8 | rec_data[1]);
	drone_mpu->temp = (raw_temp / 340.0) + 36.53;
}

// finds and stores error in gyroscope measurements(~100ms)
static void calibrate_mpu6050() {
	gyro drone_gyro = {0,0,0};
	accel drone_accel = {0,0,0};

	mpu test_mpu = {
		&drone_gyro,
		&drone_accel
	};

	float error_sum_x = 0;
	float error_sum_y = 0;
	float error_sum_z = 0;

	for(int i = 0 ; i < 50 ; i++) {
		HAL_Delay(10); //take 0.5 seconds timeout
		read_mpu6050(&test_mpu);
		error_sum_x += test_mpu.mpu_gyro->x;
		error_sum_y += test_mpu.mpu_gyro->y;
		error_sum_z += test_mpu.mpu_gyro->z;
	}
	gyro_error_x = error_sum_x / 50;
	gyro_error_y = error_sum_y / 50;
	gyro_error_z = error_sum_z / 50;
}

#ifdef IMU_DEBUG
static void mpu6050_debug(mpu* drone_mpu) {
	// test buffers
	char accel_x[30];
	char accel_y[30];
  	char accel_z[30];
  	char gyro_x[30];
  	char gyro_y[30];
  	char gyro_z[30];
  	char mpu_temp[30];

  	// write to message buffers
	CDC_Transmit_FS((uint8_t*)"Current Position:\n\r", strlen("Current Position:\n\r"));
	sprintf(accel_x, "Accel X: %.2f g\n\r", drone_mpu->mpu_accel->x);
	sprintf(accel_y, "Accel Y: %.2f g\n\r", drone_mpu->mpu_accel->y);
	sprintf(accel_z, "Accel Z: %.2f g\n\n\r", drone_mpu->mpu_accel->z);

	sprintf(gyro_x, "Gyro X: %.2f dps\n\r", drone_mpu->mpu_gyro->x);
	sprintf(gyro_y, "Gyro Y: %.2f dps\n\r", drone_mpu->mpu_gyro->y);
	sprintf(gyro_z, "Gyro Z: %.2f dps\n\r", drone_mpu->mpu_gyro->z);
	sprintf(mpu_temp, "MPU Temp: %.2f \n\n\n\n\r", drone_mpu->temp);

	// write position data to COM port
	CDC_Transmit_FS((uint8_t*)accel_x, strlen(accel_x));
	CDC_Transmit_FS((uint8_t*)accel_y, strlen(accel_y));
	CDC_Transmit_FS((uint8_t*)accel_z, strlen(accel_z));

	CDC_Transmit_FS((uint8_t*)gyro_x, strlen(gyro_x));
	CDC_Transmit_FS((uint8_t*)gyro_y, strlen(gyro_y));
	CDC_Transmit_FS((uint8_t*)gyro_z, strlen(gyro_z));

	CDC_Transmit_FS((uint8_t*)mpu_temp, strlen(mpu_temp));

}
#endif /* IMU_DEBUG */

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

		calibrate_mpu6050(); // calibrate gyro readings

		return true;
	}

	return false;
}

void read_mpu6050(mpu* drone_mpu) {
	read_accel(drone_mpu->mpu_accel);
	read_gyro(drone_mpu->mpu_gyro);
	read_temp(drone_mpu);
#ifdef IMU_DEBUG
	mpu6050_debug(mpu* drone_mpu);
#endif /* IMU_DEBUG */
}
