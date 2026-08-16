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
#include <stdio.h>
#include <stdlib.h>
#include "can.h"
#include "string.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WIND_SENSOR_CAN_ID 0x041
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef handle_GPDMA1_Channel15;

/* USER CODE BEGIN PV */

#define EZO_pH_I2C_ADDR (0x63 << 1)
#define EZO_EC_I2C_ADDR (0x64 << 1)
#define EZO_RTD_I2C_ADDR (0x66 << 1)

char uart_buffer[64];
volatile uint32_t max600_clock = 0;
volatile uint32_t max800_clock = 0;
volatile uint32_t last_reset_time = 0;
volatile uint32_t last_wind_msg_tick = 0;
volatile uint8_t consecutive_i2c_errors = 0;

// Tracks whether the "R" command was successfully ACK'd by each sensor
volatile bool rtd_cmd_ok = false;
volatile bool ec_cmd_ok = false;
volatile bool ph_cmd_ok = false;

static void MX_USART1_UART_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
int read_rtd();
int read_ec();
int read_ph();
bool SendSensorCommand(const char *cmd, uint16_t address);
void ReadSensorResponse(char *buffer, uint8_t len, uint16_t address);
float simple_atof(char *str);
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
  MX_USART1_UART_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_FDCAN1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  CAN_Init(&hfdcan1);
  NMEA0183 * windsensor = NMEA0183__create(&huart2);
  uint8_t output_wind_data[4];

  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);


  printf("It gets to here\r\n");
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);

  max600_clock = 0;
  max800_clock = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	HAL_Delay(10);

	HAL_IWDG_Refresh(&hiwdg);
	Monitor_and_Recover_CAN();
	//HAL_Delay(100); // Give the sensor a moment to wake up
	uint8_t itemsInBuffer = NMEA0183__itemsInBuffer(windsensor);
	if (itemsInBuffer == 0)
		//printf("No items in buffer.\r\n");
		printf("\r");

	else {
	  	for(uint8_t itemIndex = 0; itemIndex < itemsInBuffer; itemIndex++){
	  		NMEA0183Raw * raw_msg = NMEA0183__getTopBufferItem(windsensor);

	  		if (raw_msg == NULL){
	  			NMEA0183__incrementReadIndex(windsensor);
	  			continue;
	  		}

	  		if (NMEA0183__checkMessage(raw_msg) != GOOD_MESSAGE){
	  			NMEA0183__incrementReadIndex(windsensor);
	  			continue;
	  		}

	  		if (NMEA0183__getScentenceType(raw_msg) == MESSAGE_MWV){
	  			uint16_t processedAngle = atof((char *)NMEA0183__getField(raw_msg, 1));
	  			uint16_t processedSpeed = atof((char *)NMEA0183__getField(raw_msg, 3)) * 10.0;

	  			last_wind_msg_tick = HAL_GetTick(); // We got data!
	  			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET); // Green LED On = Healthy
	  			printf("Wind Dir: %u Wind Speed: %u\r\n", processedAngle,processedSpeed);

	  			output_wind_data[0] = (uint8_t) (processedAngle & 0xFF);
	  			output_wind_data[1] = (uint8_t) ((processedAngle >> 8) & 0xFF);
	  			output_wind_data[2] = (uint8_t) (processedSpeed & 0xFF);
	  			output_wind_data[3] = (uint8_t) ((processedSpeed >> 8) & 0xFF);
	  			if (CAN_Transmit(WIND_SENSOR_CAN_ID, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_4, output_wind_data, &hfdcan1) != HAL_OK) {
					printf("Wind sensor CAN transmit error detected");
				}

	  		}

	  		NMEA0183__incrementReadIndex(windsensor);

	  	}
	}

	uint8_t TxData_temp[3];
	memset(TxData_temp, 0, 3);
	uint8_t TxData_ec[4]; memset(TxData_ec, 0, 4);
	uint8_t TxData_ph[2]; memset(TxData_ph, 0, 2);

	// TEMPERATURE
	int temp_val = read_rtd();

	TxData_temp[0] = (uint8_t)(temp_val & 0xFF);
	TxData_temp[1] = (uint8_t)((temp_val >> 8) & 0xFF);
	TxData_temp[2] = (uint8_t)((temp_val >> 16) & 0xFF);



	if (temp_val != -1) {
		printf("Temp val: %u", temp_val);
	    if (CAN_Transmit(0x100, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_3, TxData_temp, &hfdcan1) != HAL_OK) {
	    	printf("Temperature sensor CAN tramsit failed");
		} else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
	}


	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
	//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
//	if (temp_val != -1) {
//		printf("Temperature Value: %u\r\n", temp_val);
//	}



	// PH
	int ph_val = read_ph();
	TxData_ph[0] = (uint8_t)(ph_val & 0xFF);
	TxData_ph[1] = (uint8_t)((ph_val >> 8) & 0xFF);



	if (ph_val != -1) {
		printf("\nph val: %u", ph_val);
		if (CAN_Transmit(0x110, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_2, TxData_ph, &hfdcan1) != HAL_OK) {
			printf("PH sensor CAN tramsit failed");
		} else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
	}


	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
//	if (ph_val != -1) {
//		printf("PH Value: %u\r\n", ph_val);
//	}


	// SALINITY
	int ec_val = read_ec();
	TxData_ec[0] = (uint8_t)(ec_val & 0xFF);
	TxData_ec[1] = (uint8_t)((ec_val >> 8) & 0xFF);
	TxData_ec[2] = (uint8_t)((ec_val >> 16) & 0xFF);
	TxData_ec[3] = (uint8_t)((ec_val >> 24) & 0xFF);

	if (ec_val != -1) {
		printf("\nEc val: %u", ec_val);
		if (CAN_Transmit(0x120, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_4, TxData_ec, &hfdcan1) != HAL_OK) {
			printf("EC sensor CAN tramsit failed");
		} else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
	}


	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

//	if (ec_val != -1) {
//		printf("Salinity Value: %u\r\n", ec_val);
//	}


	max600_clock++;
	max800_clock++;

	HAL_Delay(1);

	uint32_t current_time = HAL_GetTick();

	// If no data for 5 seconds (5000ms)
	if (current_time - last_wind_msg_tick > 5000) {
	    printf("ALERT: Wind Sensor Disconnected!\r\n");
	    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET); // Green LED Off
	    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); // Flash Red
	}

	// Check if Wind Sensor is dead (> 10 seconds silence)
	if ((current_time - last_wind_msg_tick > 10000) && (current_time - last_reset_time > 20000)) {
	    printf("WATCHDOG: Wind sensor dead. Performing Hard Reset...\r\n");

	    NVIC_SystemReset();


	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);


	    HAL_Delay(500);


	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

	    last_wind_msg_tick = HAL_GetTick();
	    last_reset_time = HAL_GetTick();
	}

	//printf("Ok something works\r\n");

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  HAL_PWREx_EnableVddIO2();

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV4;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 16;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 16;
  hfdcan1.Init.DataTimeSeg1 = 23;
  hfdcan1.Init.DataTimeSeg2 = 16;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel15_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30909DEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  hiwdg.Init.EWI = 0;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 4800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 LED_RED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Recover_I2C_Bus(void) {
    printf("I2C Critical Failure! Resetting Peripheral...\r\n");
    HAL_I2C_DeInit(&hi2c2);
    HAL_Delay(10);
    MX_I2C2_Init();
}

int read_rtd() {

	char response[32] = {0};

	if (max600_clock == 1) {
	    rtd_cmd_ok = SendSensorCommand("R", EZO_RTD_I2C_ADDR);
	}

	if (max600_clock == 600) {
	    if (!rtd_cmd_ok) return -1;  // Command was NACK'd, sensor likely unplugged
	    ReadSensorResponse(response, sizeof(response), EZO_RTD_I2C_ADDR);
	    if (response[0] != 1) return -1;  // EZO status byte: 1 = success
	    float response_f = simple_atof((char *) &response[1]) + 273.15;
	    return response_f * 1000;
	}

	return -1;

}

/*
 * EC Read */
int read_ec() { //0.07 -> 500,000; resolution decreases as conductivity increases

	char response[32] = {0};

	if (max600_clock == 1) {
	    ec_cmd_ok = SendSensorCommand("R", EZO_EC_I2C_ADDR);
	}

	if (max600_clock == 600) {
	    max600_clock = 0;
	    if (!ec_cmd_ok) return -1;  // Command was NACK'd, sensor likely unplugged
	    ReadSensorResponse(response, sizeof(response), EZO_EC_I2C_ADDR);
	    if (response[0] != 1) return -1;  // EZO status byte: 1 = success

	    float response_f = simple_atof(&response[1]);
	    return response_f;
	}

	return -1;

}

/*
 * pH Read */
int read_ph() { //0.001 -> 14.000, returns 1-> 14000

	char response[32] = {0};

	if (max800_clock == 1) {
	    ph_cmd_ok = SendSensorCommand("R", EZO_pH_I2C_ADDR);
	}

	if (max800_clock == 800) {
	    max800_clock = 0;
	    if (!ph_cmd_ok) return -1;  // Command was NACK'd, sensor likely unplugged
	    ReadSensorResponse(response, sizeof(response), EZO_pH_I2C_ADDR);
	    if (response[0] != 1) return -1;  // EZO status byte: 1 = success

	    float response_f = simple_atof(&response[1]);
	    return response_f * 1000;
	}

	return -1;

}

bool SendSensorCommand(const char *cmd, uint16_t address) {
    // 1. Try to transmit
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, address, (uint8_t *)cmd, strlen(cmd), 100);

    // 2. Check the result
    if (status != HAL_OK) {
        // FAILURE CASE
        consecutive_i2c_errors++; // Count the error

        // If we failed 5 times in a row, kick the hardware
        if (consecutive_i2c_errors >= 5) {
            Recover_I2C_Bus();
            consecutive_i2c_errors = 0; // Reset counter after kicking
        }
        return false;
    }
    else {
        // SUCCESS CASE
        consecutive_i2c_errors = 0;
        return true;
    }
}

void ReadSensorResponse(char *buffer, uint8_t len, uint16_t address) {
    HAL_I2C_Master_Receive(&hi2c2, address, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

void Monitor_and_Recover_CAN(void) {
    FDCAN_ProtocolStatusTypeDef ProtocolStatus;

    // 1. Get the current status of the CAN bus
    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ProtocolStatus);

    // 2. Check if we are in "Bus Off" state (Total Failure)
    // The hardware automatically turns off to protect the bus if too many errors occur.
    if (ProtocolStatus.BusOff) {
        printf("CAN Error: Bus Off detected! Resetting CAN...\r\n");

        // Force a restart of the CAN peripheral
        HAL_FDCAN_Stop(&hfdcan1);
        HAL_Delay(10); // Brief pause
        HAL_FDCAN_Start(&hfdcan1);
        return;
    }

    // 3. Check if the Outbox (Tx FIFO) is stuck/full
    // If free level is 0, the queue is full.
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
        printf("CAN Error: Tx Queue Full! Flushing old messages...\r\n");

        // Cancel all pending messages (flush the toilet) so new data can get in
        HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
    }
}

float simple_atof(char *str) {
    float result = 0.0f;
    float sign = 1.0f;
    float frac = 0.1f;
    bool after_decimal = false;
    bool started = false;

    // Skip everything until a digit or sign appears
    while (*str && !((*str >= '0' && *str <= '9') || *str == '-' || *str == '+')) {
        str++;
    }

    // Handle sign
    if (*str == '-') {
        sign = -1.0f;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // Parse number
    while (*str) {
        if (*str >= '0' && *str <= '9') {
            started = true;
            if (!after_decimal) {
                result = result * 10.0f + (*str - '0');
            } else {
                result += (*str - '0') * frac;
                frac *= 0.1f;
            }
        } else if (*str == '.') {
            after_decimal = true;
        } else {
            if (started) break;
        }
        str++;
    }

    return result * sign;
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

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
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

    // Try to print (This will only work if we moved UART1 Init to the top!)
  printf("CRITICAL FAILURE! Error_Handler.\r\n");
  //printf("reset\r\n");
  NVIC_SystemReset();

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
