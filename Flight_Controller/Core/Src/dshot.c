/*
 * dshot.c
 *
 *  Created on: May 15, 2024
 *      Author: David Exinor
 *      Reference: mokhwasomssi
 */

#include "dshot.h"

/* DMA buffer definitions */

static uint32_t motor0_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

/* Static Helper Function Prototypes */

// dshot_init

static uint32_t dshot_choose_type(dshot_type_e dshot_type);
static void dshot_set_timer(dshot_type_e dhsot_type);
static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma);
static void dshot_put_tc_callback_function();
static void dshot_start_pwm();

// dshot_tx

static uint16_t dshot_prepare_packet(uint16_t value); // might be redundant ///////////////////////////
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value);
static void dshot_prepare_dmabuffer_all(uint16_t* motor_values);
static void dshot_dma_start();
static void dshot_enable_dma_request();

/* Static Function Definitions */

static uint32_t dshot_choose_type(dshot_type_e dshot_type) {
	switch(dshot_type) {
	case DSHOT150:
		return (uint32_t)DSHOT150_HZ;
	case DSHOT300:
		return (uint32_t)DSHOT300_HZ;
	case DSHOT600:
		return (uint32_t)DSHOT600_HZ;
	default:
		return (uint32_t)0;
	}
}

static void dshot_set_timer(dshot_type_e dshot_type) {
	uint16_t dshot_prescaler;
	uint32_t timer_clk = TIMER_CLOCK; // Might need to do additional clock configurations

	// Calculate pre-scaler by dshot_type
	dshot_prescaler = lrintf((float)timer_clk / dshot_choose_type(dshot_type) + 0.01f) - 1;

	// motor 0
	__HAL_TIM_SET_PRESCALER(MOTOR_0_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_0_TIM, MOTOR_BIT_LENGTH);

	// motor 1
	__HAL_TIM_SET_PRESCALER(MOTOR_1_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_1_TIM, MOTOR_BIT_LENGTH);

	// motor 2
	__HAL_TIM_SET_PRESCALER(MOTOR_2_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_2_TIM, MOTOR_BIT_LENGTH);

	// motor 3
	__HAL_TIM_SET_PRESCALER(MOTOR_3_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_3_TIM, MOTOR_BIT_LENGTH);
}

static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {

	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}

static void dshot_put_tc_callback_function() {
	// TIM_DMA_ID_CCx depends on timer channel
	MOTOR_0_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_1_TIM->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_2_TIM->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_3_TIM->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;
}

/*
 * Start the timer channels.
 * Note: Enabling/disabling DMA request will restart a new cycle without PWM start/stop
 */
static void dshot_start_pwm() {
	HAL_TIM_PWM_Start(MOTOR_0_TIM, MOTOR_0_TIM_CHNL);
	HAL_TIM_PWM_Start(MOTOR_1_TIM, MOTOR_1_TIM_CHNL);
	HAL_TIM_PWM_Start(MOTOR_2_TIM, MOTOR_2_TIM_CHNL);
	HAL_TIM_PWM_Start(MOTOR_3_TIM, MOTOR_3_TIM_CHNL);

}

static uint16_t dshot_prepare_packet(uint16_t value) {
	uint16_t packet;
	bool dshot_telemetry = false;
	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0 ; i < 4 ; i++) {
		csum ^= csum_data; // check each nibble
		csum_data >>= 4;
	}
	csum &= 0xF;
	packet = (packet << 4) | csum;

	return packet;
}

// convert 16 bit packet to 16 pwm signals
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value) {
	uint16_t packet;
	packet = dshot_prepare_packet(value);

	for (int i = 0 ; i < 16 ; i++) {
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	motor_dmabuffer[16] = 0;
	motor_dmabuffer[17] = 0;
}

static void dshot_prepare_dmabuffer_all(uint16_t* motor_values) {
	dshot_prepare_dmabuffer(motor0_dmabuffer, motor_values[0]);
	dshot_prepare_dmabuffer(motor1_dmabuffer, motor_values[1]);
	dshot_prepare_dmabuffer(motor2_dmabuffer, motor_values[2]);
	dshot_prepare_dmabuffer(motor3_dmabuffer, motor_values[3]);
}

static void dshot_dma_start() {
	HAL_DMA_Start_IT(MOTOR_0_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t)motor0_dmabuffer,
			(uint32_t)&MOTOR_0_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);

	HAL_DMA_Start_IT(MOTOR_1_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t)motor1_dmabuffer,
				(uint32_t)&MOTOR_1_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);

	HAL_DMA_Start_IT(MOTOR_2_TIM->hdma[TIM_DMA_ID_CC3], (uint32_t)motor2_dmabuffer,
				(uint32_t)&MOTOR_2_TIM->Instance->CCR3, DSHOT_DMA_BUFFER_SIZE);

	HAL_DMA_Start_IT(MOTOR_3_TIM->hdma[TIM_DMA_ID_CC4], (uint32_t)motor3_dmabuffer,
				(uint32_t)&MOTOR_3_TIM->Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);
}

static void dshot_enable_dma_request() {
	__HAL_TIM_ENABLE_DMA(MOTOR_0_TIM, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(MOTOR_1_TIM, TIM_DMA_CC2);
	__HAL_TIM_ENABLE_DMA(MOTOR_2_TIM, TIM_DMA_CC3);
	__HAL_TIM_ENABLE_DMA(MOTOR_3_TIM, TIM_DMA_CC4);
}

/* Function Definitions */

void dshot_init(dshot_type_e dshot_type) {
	dshot_set_timer(dshot_type);
	dshot_put_tc_callback_function();
	dshot_start_pwm();
}

void dshot_tx(double* motor_values) {
	dshot_prepare_dmabuffer_all(motor_values);
	dshot_dma_start();
	dshot_enable_dma_request();
}
