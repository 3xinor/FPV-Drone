/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "ibus.h"
#include "imu.h"
#include "dshot.h"
#include "PID_Control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Ibus
static uint8_t iBus_rx_buffer[IBUS_BUFFER_SIZE];
static ibus_rx_t ibus_rx_struct = {0};
static bool wakeup = false;

//MPU6050
static gyro drone_gyro = {0,0,0,0,0,0};
static accel drone_accel = {0,0,0,0,0,0};
static mpu drone_mpu = {
	&drone_gyro,
	&drone_accel
};
static uint8_t tries = 0;

// ESCs
static uint16_t motor_values[4] = {0,0,0,0};



// PID controller stuff



// Max attitude rate of change rates (degrees per second)
float max_rate_roll = 30.0;
float max_rate_pitch = 30.0;
float max_rate_yaw = 50.0;

// Desired Flight Controller Cycle Time
float target_cycle_hz = 250.0;

// PID Controller values
const float pid_roll_kp = 0.00043714285;
const float pid_roll_ki = 0.00255;
const float pid_roll_kd = 0.00002571429;
const float pid_pitch_kp = pid_roll_kp;
const float pid_pitch_ki = pid_roll_ki;
const float pid_pitch_kd = pid_roll_kd;
const float pid_yaw_kp = 0.001714287;
const float pid_yaw_ki = 0.003428571;
const float pid_yaw_kd = 0.0;
const float i_limit = 150.0;

// throttle settings
double throttle_idle = 0.05;
double throttle_govenor = 0.25;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /********************* State 1 : Wakeup *********************/

	  // Wait for user to turn SWA on

	  while (wakeup == false) {
		  // check if new data has been received from Controller
		  if(HAL_UART_Receive(&huart1, iBus_rx_buffer, IBUS_BUFFER_SIZE, 1000) == HAL_OK) {
			  // Check if the received data is a valid iBus packet
			  if (iBus_rx_buffer[1] == 0x20) {
				  // Process the iBus packet
				  process_iBus_data(iBus_rx_buffer, &ibus_rx_struct);

				  // check if SWA was turned on
				  if (ibus_rx_struct.ch5 > 0.5) {wakeup = true;}
			  }
		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // set LED ON
		  HAL_Delay(1000); // 1 second timeout
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // set LED OFF
	  }


	  /********************* State 2 : Prepare Drone for Flight *********************/


	  // Initialize MPU 6050
	  while (init_mpu6050() != true) {
		  tries++;
		  if (tries >= 30) {
			  signal_init_fail();
			  break;
		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // set LED ON
		  HAL_Delay(1000); // 1 second timeout
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // set LED OFF
	  }
	  // failed to init MPU6050 retry setup
	  if (tries >= 30) {
		  tries = 0;
		  continue;
	  }

	  // calibrate mpu6050
	  calibrate_mpu6050(&drone_mpu);

	  // Initialize and arm motors
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // set LED OFF
	  dshot_init(DSHOT600);
	  HAL_Delay(500);
	  dshot_arm_motors(motor_values);

	  /********************* State 3: PID Control *********************/

	  // constraints calculations / state variables

	  float cycle_time_seconds = 1.0 / target_cycle_hz;
	  uint16_t cycle_time_us = (uint16_t)(cycle_time_seconds * 1000000);
	  double max_throttle = throttle_govenor;
	  double throttle_range = max_throttle - throttle_idle;

	  // PID inits
	  PIDController_t roll_PID = {0};
	  PIDController_t pitch_PID = {0};
	  PIDController_t yaw_PID = {0};

	  initPIDController(&roll_PID, pid_roll_kp, pid_roll_ki, pid_roll_kd, cycle_time_seconds, i_limit);
	  initPIDController(&pitch_PID, pid_pitch_kp, pid_pitch_ki, pid_pitch_kd, cycle_time_seconds, i_limit);
	  initPIDController(&yaw_PID, pid_yaw_kp, pid_yaw_ki, pid_yaw_kd, cycle_time_seconds, i_limit);

	  // PID Controller Timing
	  uint16_t start_time;
	  uint16_t end_time;

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // set LED ON

	  while(1) { // Change to check if SWA or SWC is pressed

		  // Mark start time (us)
		  start_time = __HAL_TIM_GET_COUNTER(&htim10);

		  // read IMU data
		  read_mpu6050(&drone_mpu);

		  // read control commands from RC
		  if(HAL_UART_Receive(&huart1, iBus_rx_buffer, IBUS_BUFFER_SIZE, 1000) == HAL_OK) {
			  // Check if the received data is a valid iBus packet
			  if (iBus_rx_buffer[1] == 0x20) {
				  // Process iBus packet
				  process_iBus_data(iBus_rx_buffer, &ibus_rx_struct);
			  }
		  }

		  // Adjust motor outputs
		  if (ibus_rx_struct.ch5 < 0.5) {
			  // turn motors off
			  motor_values[0] = 0;
			  motor_values[1] = 0;
			  motor_values[2] = 0;
			  motor_values[3] = 0;
			  dshot_write(motor_values);

			  // reset PID's
			  resetPID(&roll_PID);
			  resetPID(&pitch_PID);
			  resetPID(&yaw_PID);

		  } else {

			  // calculate the adjusted desired throttle ( above idle, below govenor, linearly scaled)
			  float adj_throttle = throttle_idle + (throttle_range * ibus_rx_struct.ch3);

			  // calculate errors - difference between the actual rates and set rates
			  float error_rate_roll = (ibus_rx_struct.ch1 * max_rate_roll) - drone_mpu.mpu_gyro->x;
			  float error_rate_pitch = (ibus_rx_struct.ch2 * max_rate_pitch) - drone_mpu.mpu_gyro->y;
			  float error_rate_yaw = (ibus_rx_struct.ch4 * max_rate_yaw) - drone_mpu.mpu_gyro->z;

			  // PID calculations
			  float pid_roll = calculatePID(&roll_PID, error_rate_roll);
			  float pid_pitch = calculatePID(&pitch_PID, error_rate_pitch);
			  float pid_yaw = calculatePID(&yaw_PID, error_rate_yaw);

			  // calculate final throttle values
			  float t1 = adj_throttle + pid_pitch + pid_roll - pid_yaw;
			  float t2 = adj_throttle + pid_pitch - pid_roll + pid_yaw;
			  float t3 = adj_throttle - pid_pitch + pid_roll + pid_yaw;
			  float t4 = adj_throttle - pid_pitch - pid_roll - pid_yaw;

			  // Write throttle values to motor ******** IDK WHAT MOTORS CORRESPOND TO WHAT RIGHT NOW ********
			  motor_values[0] = (uint16_t)(t1 * 2000);
			  motor_values[1] = (uint16_t)(t2 * 2000);
			  motor_values[2] = (uint16_t)(t3 * 2000);
			  motor_values[3] = (uint16_t)(t4 * 2000);
			  dshot_write(motor_values);
		  }

		  // Mark end time (us)
		  end_time = __HAL_TIM_GET_COUNTER(&htim10);

		  // wait before continuing to achieve desired system frequency
		  uint16_t elapsed_time = end_time - start_time;
		  if (elapsed_time < cycle_time_us) {
			  // run empty clock cycles given 84MHz system clock
			  uint16_t burn_clock_cycles = (cycle_time_us - elapsed_time) * 84;
			  for ( ; burn_clock_cycles > 0 ; burn_clock_cycles--){}
		  }


	  }

	  /********************* State 4: DeInit *********************/

	  /********************* State Emergency Shutdown: Kill Drone *********************/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
