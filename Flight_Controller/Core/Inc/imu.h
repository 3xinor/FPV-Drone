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

//#define IMU_DEBUG

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
}accel;

typedef struct {
	float x;
	float y;
	float z;
}gyro;

typedef struct {
	gyro* mpu_gyro;
	accel* mpu_accel;
	float temp;
}mpu;


/*
 * Initializes the mpu6050 device
 * Returns:
 * 		true if successful, false otherwise
 */
bool init_mpu6050();

/*
 * reads mpu raw data and returns 3-axis gyroscope, 3 axis accelerometer values, and temperature
 * in the form of a simplified struct with units {degrees per second, acceleration in g, and degrees Celsius}
 * respectively.
 */
void read_mpu6050(mpu* drone_mpu);


#endif /* INC_IMU_H_ */
