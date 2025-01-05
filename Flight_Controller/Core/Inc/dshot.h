/*
 * dshot.h
 *
 *  Created on: May 15, 2024
 *      Author: David Exinor
 *      Reference: mokhwasomssi
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "tim.h"
#include <stdbool.h>
#include <math.h>

/* User Configuration */

// Timer Clock
#define TIMER_CLOCK				100000000 // 100MHz

/* DSHOT ESC Peripherals */

// MOTOR 0 (PA0) - TIM2 Channel 1, DMA1 Stream 5
#define MOTOR_0_TIM				(&htim2)
#define MOTOR_0_TIM_CHNL		TIM_CHANNEL_1

// MOTOR 1 (PA1) - TIM5 Channel 2, DMA1 Stream 4
#define MOTOR_1_TIM				(&htim5)
#define MOTOR_1_TIM_CHNL		TIM_CHANNEL_2

// MOTOR 2 (PA2) - TIM2 Channel 3, DMA1 Stream 1
#define MOTOR_2_TIM				(&htim2)
#define MOTOR_2_TIM_CHNL		TIM_CHANNEL_3

// MOTOR 3 (PA3)  - TIM5 Channel 4, DMA1 Stream 3
#define MOTOR_3_TIM				(&htim5)
#define MOTOR_3_TIM_CHNL		TIM_CHANNEL_4

/* DSHOT Definitions */

#define MHZ_TO_HZ(x)			((x) * 1000000)

#define DSHOT150_HZ     		MHZ_TO_HZ(3)
#define DSHOT300_HZ     		MHZ_TO_HZ(6)
#define DSHOT600_HZ     		MHZ_TO_HZ(12)

#define MOTOR_BIT_0				7
#define MOTOR_BIT_1				14
#define MOTOR_BIT_LENGTH		20

#define DSHOT_FRAME_SIZE		16
#define	DSHOT_DMA_BUFFER_SIZE 	18 // resolution + frame reset (2us)

#define DSHOT_MIN_THROTTLE		48
#define DHSOT_MAX_THROTTLE		2047
#define DHSOT_RANGE				(DHSOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

typedef enum {
	DSHOT150,
	DSHOT300,
	DSHOT600
} dshot_type_e;

/* DSHOT Functions */

/*
 * Initializes the esc's connected to stm32 mc with desired communication protocol
 * Arg: dshot_type - a enum describing which dshot protocol to use for initialization
 */
void dshot_init(dshot_type_e dhsot_type);

/*
 * Writes the desired motor speeds to each esc
 */
void dshot_tx(double* motor_values);

#endif /* INC_DSHOT_H_ */
