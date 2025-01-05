/*
 * dshot.h
 *
 *  Created on: Aug 27, 2024
 *      Author: Dave
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "tim.h"
#include <stdbool.h>
#include <math.h>

/*  User Configuration */

// Timer Clock
#define TIMER_CLOCK					100000000 //100MHz

// Motor 1 (PA3) Timer 5 Channel 4, DMA1 Stream 3
#define MOTOR_1_TIM					(&htim5)
#define MOTOR_1_TIM_CHANNEL			TIM_CHANNEL_4

// Motor 2 (PA2) Timer 2 Channel 3, DMA1 Stream 1
#define MOTOR_2_TIM 				(&htim2)
#define MOTOR_2_TIM_CHANNEL 		TIM_CHANNEL_3

// Motor 3 (PA1) Timer 5 Channel 2, DMA1 Stream 4
#define MOTOR_3_TIM 				(&htim5)
#define MOTOR_3_TIM_CHANNEL 		TIM_CHANNEL_2

// Motor 4 (PA0) Timer 2 Channel 1, DMA1 Stream 5
#define MOTOR_4_TIM					(&htim2)
#define MOTOR_4_TIM_CHANNEL 		TIM_CHANNEL_1

/* Definitions */

#define MHZ_TO_HZ(x) 				((x) * 1000000)

#define DSHOT600_HZ					MHZ_TO_HZ(12)
#define DSHOT300_HZ					MHZ_TO_HZ(6)
#define DSHOT150_HZ 				MHZ_TO_HZ(3)

#define MOTOR_BIT_0 				7
#define MOTOR_BIT_1					14
#define MOTOR_BIT_LENGTH			20

#define DSHOT_FRAME_SIZE			16
#define DSHOT_DMA_BUFFER_SIZE		18 // resolution + frame reset(2us)

#define DSHOT_MIN_THROTTLE			48
#define DSHOT_MAX_THROTTLE 			2047

#define DSHOT_RANGE			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

/* Enumeration */

typedef enum {
	DSHOT150,
	DSHOT300,
	DSHOT600
} dshot_type_e;

/* Functions */

void dshot_init(dshot_type_e dshot_type);

void dshot_write(uint16_t* motor_value);

void dshot_arm_motors(uint16_t *motor_value);


#endif /* INC_DSHOT_H_ */
