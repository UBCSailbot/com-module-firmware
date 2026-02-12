/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "CANSPI.h"
#include "CANSERVO.h"
#include "stdio.h"

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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t g_servo_step = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void delay_step(const char *msg)
{
  printf("\r\n--- %s (wait 5s) ---\r\n", msg);
  HAL_Delay(5000);
}

static void set_limits_wide(void)
{
  // Wide soft limits: allow full 0..16383 range
  set_servo_min_position_limit(0);
  set_servo_max_position_limit(16383);
  HAL_Delay(50);
}

static void check_emergency_status(void)
{
    uint16_t estop = receive_can_msg(REG_EMERGENCY_STOP);

    printf("REG_EMERGENCY_STOP = 0x%04X\r\n", estop);

    if (estop == 0)
    {
        printf("Status: OK (no emergency flags)\r\n");
    }
    else
    {
        printf("Status: EMERGENCY ACTIVE!\r\n");
        printf("→ Servo will likely ignore motion commands\r\n");
    }
}

void clear_emergency_via_powercfg(void)
{
    uint16_t val = 0;

    // Disable forced emergency mode bits
    val &= ~PWR_FORCED_ES_MASK;

    send_can_msg(val, REG_POWER_CONFIG);

    HAL_Delay(50);

    uint16_t st = receive_can_msg(REG_EMERGENCY_STOP);
    printf("After powercfg clear: ESTOP=0x%04X\r\n", st);
}

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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();


  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* USER CODE BEGIN 2 */
    CANSPI_Initialize();

    //HAL_Delay(5000);
    servo_init();



//    if (!SERVO_OK()) {
//  	  //HAL_TIM_Base_Start_IT(&htim2);
//  	  //servo_set_speed_down_emergeny_mode();
//  	  printf("CAN Servo operating outside of nominal conditions upon bootup. One hour timer started...");
//    }


    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  printf("\r\n===== MIDPOINT NON-ZERO TEST (+30 deg) =====\r\n");

	      servo_init();
	      HAL_Delay(1000);

	      // Move to a non-zero angle first
	      printf("\r\nMove to +30 deg (old reference)\r\n");
	      set_servo_angle(30.0f);
	      HAL_Delay(3000);

	      uint16_t raw_before = receive_can_msg(REG_POSITION_NEW);
	      printf("Raw before re-zero = 0x%04X\r\n", (unsigned)raw_before);

	      // Re-zero here (THIS becomes new 0°)
	      printf("\r\nSet midpoint at current position (should become new 0 deg)\r\n");
	      set_servo_midpoint();
	      HAL_Delay(500);

	      printf("New midpoint = 0x%04X\r\n", (unsigned)servo_midpoint);

	      // Now command 0° (should stay basically where it is)
	      printf("\r\nCommand 0 deg (new reference) - should stay near same physical position\r\n");
	      set_servo_angle(0.0f);
	      HAL_Delay(3000);

	      uint16_t raw_after0 = receive_can_msg(REG_POSITION_NEW);
	      printf("Raw after cmd 0 (new ref) = 0x%04X\r\n", (unsigned)raw_after0);

	      // Sanity: check reported angle should be near 0
	      float a0 = check_servo_angle();
	      printf("check_servo_angle() after cmd 0 = %.2f deg\r\n", a0);

	      // Command +60° new ref
	      printf("\r\nCommand +60 deg (new reference)\r\n");
	      set_servo_angle(60.0f);
	      HAL_Delay(3000);

	      uint16_t raw_after60 = receive_can_msg(REG_POSITION_NEW);
	      printf("Raw after +60 = 0x%04X\r\n", (unsigned)raw_after60);

	      float a60 = check_servo_angle();
	      printf("check_servo_angle() after +60 = %.2f deg\r\n", a60);

	      // Print limits
	      uint16_t min_lim = receive_can_msg(REG_POSITION_MIN_LIMIT);
	      uint16_t max_lim = receive_can_msg(REG_POSITION_MAX_LIMIT);
	      printf("\r\nLimits now: MIN=0x%04X  MAX=0x%04X\r\n",
	             (unsigned)min_lim, (unsigned)max_lim);

	      // Emergency status
	      uint16_t estop = receive_can_msg(REG_EMERGENCY_STOP);
	      printf("Emergency status = 0x%04X\r\n", (unsigned)estop);

	      printf("\r\n===== TEST COMPLETE =====\r\n");




	  //servo_soft_reset_only();
//	  uint16_t power = receive_can_msg(REG_POWER_CONFIG);
//	  printf("Power_config: %x",power);
//	  set_servo_angle(0);
//	  uint16_t max_voltage = receive_can_msg(REG_VOLTAGE_MAX);
//	  printf("Max volt: %u",max_voltage);
//	  HAL_Delay(3000);

//	  servo_soft_reset_only();
//	  servo_clear_forced_es();
//	  set_servo_max_voltage(SERVO_MAX_VOLTAGE);
//	  set_servo_min_voltage(SERVO_MIN_VOLTAGE);
//	  set_servo_angle (90);
//	  HAL_Delay(3000);
//	  uint16_t power = receive_can_msg(REG_POWER_CONFIG);
//	  uint16_t emergency = receive_can_msg(REG_EMERGENCY_STOP);
//	  printf("Power_config: 0x%04\r\n",power);
//	  printf("Emergency: 0x%04\r\n",emergency);
//	  HAL_Delay(3000);
//	  servo_clear_forced_es();
//	  HAL_Delay(3000);







	  // =========================
	  // TEST 6: Voltage thresholds -> emergency flags
	  // =========================
//	  printf("\r\n[TEST 6] Voltage min/max thresholds trigger emergency flags\r\n");
//	  printf("Objective: Flags assert when supply is outside window.\r\n");
//
//	  // Narrow window around nominal 12.00V
//	  set_servo_min_voltage(1190);  // 11.90V
//	  set_servo_max_voltage(1210);  // 12.10V
//	  //servo_soft_reset();
//	  delay_step("Set VOLT_MIN=11.90V (1190), VOLT_MAX=12.10V (1210)");
//
//	  // Step 1: nominal
//	  printf("Set PSU to 12.00V. Reading REG_EMERGENCY_STOP...\r\n");
//	  uint16_t st = receive_can_msg(REG_EMERGENCY_STOP);
//	  printf("REG_EMERGENCY_STOP = 0x%04X\r\n", st);
//	  delay_step("Expected: No voltage flags");
//
//	  // Step 2: undervoltage
//	  printf("Set PSU to 11.80V. Reading REG_EMERGENCY_STOP...\r\n");
//	  st = receive_can_msg(REG_EMERGENCY_STOP);
//	  printf("REG_EMERGENCY_STOP = 0x%04X\r\n", st);
//	  delay_step("Expected: Undervoltage flag(s) set");
//	  uint16_t estop = receive_can_msg(0x46);
//	  printf("estop = 0x%04X\r\n", estop);
//
//	  // Step 3: overvoltage
//	  printf("Set PSU to 12.20V. Reading REG_EMERGENCY_STOP...\r\n");
//	  st = receive_can_msg(REG_EMERGENCY_STOP);
//	  printf("REG_EMERGENCY_STOP = 0x%04X\r\n", st);
//	  delay_step("Expected: Overvoltage flag(s) set");
//
//	  // Step 4: back to nominal
//	  printf("Return PSU to 12.00V. Reading REG_EMERGENCY_STOP...\r\n");
//	  st = receive_can_msg(REG_EMERGENCY_STOP);
//	  printf("REG_EMERGENCY_STOP = 0x%04X\r\n", st);
//	  delay_step("Expected: Flags clear (if auto-clear), otherwise document latch behavior");

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
//	if (htim->Instance == TIM2) {
//
//		servo_soft_reset_only();
//
//		if (!SERVO_OK()) {
//			printf("Board still fails after 1hr wait time. Board resetting...");
//			return;
//
//		} else {
//			servo_clear_forced_es();
//			HAL_TIM_Base_Stop_IT(&htim2);
//		}
//
//	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
