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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include "imu.h"
#include "usbd_cdc_if.h"
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

  uint16_t motor0_speed = 0; // up-to 2000
  uint16_t motor1_speed = 0; // up-to 2000
  uint16_t motor2_speed = 0; // up-to 2000
  uint16_t motor3_speed = 0; // up-to 2000
  uint16_t motors[4] = {motor0_speed, motor1_speed, motor2_speed, motor3_speed};

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // initialize mpu
  bool mpu_init = init_mpu6050();

  gyro drone_gyro = {0,0,0};
  accel drone_accel = {0,0,0};

  mpu drone_mpu = {
	  &drone_gyro,
	  &drone_accel
  };

  // initialize dshot protocol for communication with esc's
  dshot_init(DSHOT300);

  ////////// Test buffers ////////////
  // message buffers for mpu position transmission over COM
  char accel_x[20];
  char accel_y[20];
  char accel_z[20];
  char gyro_x[20];
  char gyro_y[20];
  char gyro_z[20];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_Delay(2000); // 2 second delay
	  if(mpu_init) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		CDC_Transmit_FS((uint8_t*)"Current Position:\n\r", strlen("Current Position:\n\r"));

		// read positional data from mpu
		read_mpu6050(&drone_mpu);

		// write to message buffers
		sprintf(accel_x, "Accel X: %.2f g\n\r", drone_mpu.mpu_accel->x);
		sprintf(accel_y, "Accel Y: %.2f g\n\r", drone_mpu.mpu_accel->y);
		sprintf(accel_z, "Accel Z: %.2f g\n\n\r", drone_mpu.mpu_accel->z);

		sprintf(gyro_x, "Gyro X: %.2f dps\n\r", drone_mpu.mpu_gyro->x);
		sprintf(gyro_y, "Gyro Y: %.2f dps\n\r", drone_mpu.mpu_gyro->y);
		sprintf(gyro_z, "Gyro Z: %.2f dps\n\n\n\n\r", drone_mpu.mpu_gyro->z);

		// write position data to COM port
		CDC_Transmit_FS((uint8_t*)accel_x, strlen(accel_x));
		CDC_Transmit_FS((uint8_t*)accel_y, strlen(accel_y));
		CDC_Transmit_FS((uint8_t*)accel_z, strlen(accel_z));

		CDC_Transmit_FS((uint8_t*)gyro_x, strlen(gyro_x));
		CDC_Transmit_FS((uint8_t*)gyro_y, strlen(gyro_y));
		CDC_Transmit_FS((uint8_t*)gyro_z, strlen(gyro_z));
	  } else {
		  CDC_Transmit_FS((uint8_t*)"Failed to initialize MPU\n\r", strlen("Failed to initialize MPU\n\r"));
		  CDC_Transmit_FS((uint8_t*)"Retrying to MPU initialization\n\r", strlen("Retrying to MPU initialization\n\r"));
		  mpu_init = init_mpu6050();
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
