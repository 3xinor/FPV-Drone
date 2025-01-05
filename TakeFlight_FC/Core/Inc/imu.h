/*
 * imu.h
 *
 *  Created on: May 15, 2024
 *      Author: David Exinor
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdbool.h>
#include "i2c.h"

#define MPU6050_ADDR			0xD0

#define SMPLRT_DIV_REG 			0x19
#define GYRO_CONFIG_REG 		0x1B
#define ACCEL_CONFIG_REG 		0x1C
#define ACCEL_XOUT_H_REG 		0x3B
#define TEMP_OUT_H_REG 			0x41
#define GYRO_XOUT_H_REG 		0x43
#define PWR_MNGMT_1_REG 			0x6B
#define WHO_AM_I_REG 			0x75


typedef struct {
	float x;
	float y;
	float z;
	float x_error;
	float y_error;
	float z_error;
}accel;

typedef struct {
	float x;
	float y;
	float z;
	float x_error;
	float y_error;
	float z_error;
}gyro;

typedef struct {
	gyro* mpu_gyro;
	accel* mpu_accel;
}mpu;


/*
 * Initializes the mpu6050 device
 * Returns:
 * 		true if successful, false otherwise
 */
bool init_mpu6050();

/*
 * reads mpu raw data and returns 3-axis gyroscope and 3 axis accelerometer values
 * in the form of a simplified struct
 */
void read_mpu6050(mpu* drone_mpu);

void calibrate_mpu6050(mpu* drone_mpu);

void signal_init_fail();

#endif /* INC_IMU_H_ */
