/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// conversion constants 
#define TEMP_C 0.00390625
#define ACCEL_G 0.061
#define GYRO_DPS 0.00875

// device registries
#define ADDR_READ 0b11010101
#define ADDR_WRITE 0b11010100

// registries for enable writing
#define CTRL_RESET_REG 0x12
#define ACCEL_CTRL_REG 0x10
#define GYRO_CTRL_REG 0x11
#define GYRO_DPS_REG 0x15
#define GYRO_FILTER_REG 0x16
#define ACCEL_FILTER_REG 0x17

// values for enable writing
#define CONF_RESET 0b00000001
#define GRYO_240HZ 0b00000111
#define GYRO_250DPS 0b00001001
#define GYRO_ENABLE_FILTER 0b00000001
#define ACCEL_240HZ 0b00000111
#define ACCEL_ENABLE_FILTER 0b00000000

// addresses for gryo values
#define ADDR_GRYO_X1 0x22
#define ADDR_GRYO_X2 0x23
#define ADDR_GRYO_Y1 0x24
#define ADDR_GRYO_Y2 0x25
#define ADDR_GRYO_Z1 0x26
#define ADDR_GRYO_Z2 0x27

// addresses for gryo values
#define ADDR_ACCEL_X1 0x28
#define ADDR_ACCEL_X2 0x29
#define ADDR_ACCEL_Y1 0x2A
#define ADDR_ACCEL_Y2 0x2B
#define ADDR_ACCEL_Z1 0x2C
#define ADDR_ACCEL_Z2 0x2D

// addresses for temperature sensor values
#define ADDR_TEMP1 0x20
#define ADDR_TEMP2 0x21

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // pointers
  uint8_t regGyroX1, regGyroX2, regGyroY1, regGyroY2, regGyroZ1, regGyroZ2;
  uint8_t regAccelX1, regAccelX2, regAccelY1, regAccelY2, regAccelZ1, regAccelZ2;
  uint8_t regTemp1, regTemp2;
  uint8_t writePointer;

  // test reg and pointer
  uint8_t regTest = 0x12;
  uint8_t readreg;

  // enable all sensors and filters
  writePointer = CONF_RESET;
  HAL_I2C_Mem_Write(&hi2c1, ADDR_WRITE, CTRL_RESET_REG, 1, &writePointer, 1, 500);
  writePointer = GRYO_240HZ;
  HAL_I2C_Mem_Write(&hi2c1, ADDR_WRITE, GYRO_CTRL_REG, 1, &writePointer, 1, 500);
  writePointer = GYRO_250DPS;
  HAL_I2C_Mem_Write(&hi2c1, ADDR_WRITE, GYRO_DPS_REG, 1, &writePointer, 1, 500);
  writePointer = GYRO_FILTER_REG;
  HAL_I2C_Mem_Write(&hi2c1, ADDR_WRITE, GYRO_FILTER_REG, 1, &writePointer, 1, 500);
  writePointer = ACCEL_CTRL_REG;
  HAL_I2C_Mem_Write(&hi2c1, ADDR_WRITE, ACCEL_240HZ, 1, &writePointer, 1, 500);
  writePointer = ACCEL_ENABLE_FILTER;
  HAL_I2C_Mem_Write(&hi2c1, ADDR_WRITE, ACCEL_FILTER_REG, 1, &writePointer, 1, 500);


  while (1)
  {
    //// GYRO VALUES
    // Read gyroscope X values then combine
	  HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_GRYO_X1, 1, &regGyroX1, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_GRYO_X2, 1, &regGyroX2, 1, 1000); 
    int16_t gyroXraw = (int16_t)(regGyroX2 << 8) | regGyroX1;

    // Read gyroscope Y values then combine
	  HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_GRYO_Y1, 1, &regGyroY1, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_GRYO_Y2, 1, &regGyroY2, 1, 1000);
    int16_t gyroYraw = (int16_t)(regGyroY2 << 8) | regGyroY1;

    // Read gyroscope Y values then combine
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_GRYO_Z1, 1, &regGyroZ1, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_GRYO_Z2, 1, &regGyroZ2, 1, 1000);
    int16_t gyroZraw = (int16_t)(regGyroZ2 << 8) | regGyroZ1;

    // Make all gyro values readable
    float gyroX=gyroXraw*GYRO_DPS;
    float gyroY=gyroYraw*GYRO_DPS;
    float gyroZ=gyroZraw*GYRO_DPS;

    // Print all gyro values
    printf("the gyro X value is %f\n\r", gyroX);
    printf("the gyro Y value is %f\n\r", gyroY);
    printf("the gyro Z value is %f\n\n\r", gyroZ);

    //// ACCELEROMETER VALUES
    // Read accelerometers X values then combine
	  HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_ACCEL_X1, 1, &regAccelX1, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_ACCEL_X2, 1, &regAccelX2, 1, 1000); 
    int16_t accelXraw = (int16_t)(regAccelX2 << 8) | regAccelX1;

    // Read accelerometers Y values then combine
	  HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_ACCEL_Y1, 1, &regAccelY1, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_ACCEL_Y2, 1, &regAccelY2, 1, 1000);
    int16_t accelYraw = (int16_t)(regAccelY2 << 8) | regAccelY1;

    // Read accelerometers Y values then combine
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_ACCEL_Z1, 1, &regAccelZ1, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_ACCEL_Z2, 1, &regAccelZ2, 1, 1000);
    int16_t accelZraw = (int16_t)(regAccelZ2 << 8) | regAccelZ1;

    // Make all accel values readable
    float accelX=accelXraw*ACCEL_G;
    float accelY=accelYraw*ACCEL_G;
    float accelZ=accelZraw*ACCEL_G;
    
    // Print all accel values
    printf("the accel X value is %f\n\r", accelX);
    printf("the accel Y value is %f\n\r", accelY);
    printf("the accel Z value is %f\n\n\n\r", accelZ);


    // Read temperature
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_TEMP1, 1, &regTemp1, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, ADDR_READ, ADDR_TEMP2, 1, &regTemp2, 1, 1000);
    int16_t tempRaw = (int16_t)(regTemp2 << 8) | regTemp1;

    // convert to human units
    float temp=tempRaw*TEMP_C+25; // this is the coversion factor dont ask

    // Print temp value
    printf("the temp value is %f\n\n\r", temp);


    // Test registry. Uncomment as needed. Nothing is enabled to accomodate. Currently viewing settings for ctrl1
	  // HAL_I2C_Mem_Read(&hi2c1, adrRead, regTest, 1, &readreg, 1, 1000);
	  // printf("The value is %x\n\n\n\r", readreg);
	  // readreg = 0;

	  HAL_Delay(2000);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
