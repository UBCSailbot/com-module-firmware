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

#include "RUDDER.h"
#include "RUDDER_PARAMS.h"
#include "RUDDER_UTILS.h"
//#include "RUDDERPID.h"
#include "BRITER.h"
#include <stdio.h>
#include <stdlib.h>
#include <can.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define ZEROING_CALIBRATION

#define CAN_TX_DELAY_MS 100
#define RUDDER_TO_MAINFRAME_DEBUG_ID 0x204
#define CONTROL_MODEL_PARAMS_ID 0x200
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef handle_GPDMA1_Channel9;
DMA_HandleTypeDef handle_GPDMA1_Channel15;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;
FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t RxData1[64];
uint8_t RxData2[64];
//HAL_StatusTypeDef CanStartStatus; Commented out arbitrarily

float desiredRudderAngle;

uint8_t * rudder_debug_frame;

//0 = auto mode, 1 = manual
uint8_t controller_mode = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_UCPD1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static uint8_t byte_to_dlc(uint8_t len);
static uint8_t dlc_to_bytes(uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

BRITER * encoderObject;
int get_encoder_delta(int prev, int curr) {
      int delta = curr - prev;
      if (delta > 512) delta -= 1024;
      if (delta < -512) delta += 1024;
      return delta;
  }

NMEA0183 *ecompass;
const char PASHR_ENABLE_CMD[] = "$JASC,PASHR,1\x0D\x0A"; //enables PASHR sentence type
const char GPHDT_FREQ[] = "$JASC,GPHDT,1\x0D\x0A"; //allows heading data received at 10Hz

void uint32_to_little_endian_bytes(uint32_t value, uint8_t bytes[4]) {
    bytes[0] = (uint8_t)(value & 0xFF);
    bytes[1] = (uint8_t)((value >> 8) & 0xFF);
    bytes[2] = (uint8_t)((value >> 16) & 0xFF);
    bytes[3] = (uint8_t)((value >> 24) & 0xFF);
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
  //CAN frame ID 0x204 tx_frame
  rudder_debug_frame = (uint8_t * ) malloc(16);

  PIDControllerFixed fixedController = getRudderFixedParams();
  initController(fixedController);

//  float desiredRudderAngle = 0.0f;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_UCPD1_Init();
  MX_ADC1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */



  //CAN Setup

    // CAN Library setup

    CAN_Init(&hfdcan1, 0x131); //put in hbid

    // /* Configure standard ID reception filter to Rx buffer 0 */
    // sFilterConfig.IdType = FDCAN_STANDARD_ID;
    // sFilterConfig.FilterIndex = 0;
    // sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // sFilterConfig.FilterID1 = 0x000;
    // sFilterConfig.FilterID2 = 0x7FF;
    // if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    // /* Configure extended ID reception filter to Rx FIFO 1 */
    // sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    // sFilterConfig.FilterIndex = 0;
    // sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
    // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    // sFilterConfig.FilterID1 = 0x1111111;
    // sFilterConfig.FilterID2 = 0x2222222;
    // if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    // /* Configure global filter:
    //    Filter all remote frames with STD and EXT ID
    //    Reject non matching frames with STD ID and EXT ID */
    // if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    // {
    //     Error_Handler();
    // }

    // /*##-2 Start FDCAN controller (continuous listening CAN bus) ##############*/
    // CanStartStatus = HAL_FDCAN_Start(&hfdcan1);
    // if (CanStartStatus != HAL_OK)
    // {
    //   Error_Handler();
    // }

    // if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    // if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    uint32_t can_frame_tx_time = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
    encoderObject = BRITER__create(&huart2, 20);
    MOTOR_CONFIG motorConfig = {
            .motorDacPeripheral = &hdac1,
            .motorDacChannel = DAC_CHANNEL_2,
            .enableGPIOPeripheral = GPIOG,
            .enableGPIOPin = GPIO_PIN_1,
            .reverseGPIOPeripheral = GPIOF,
            .reverseGPIOPin = GPIO_PIN_13
      };
    Setup_Motor(motorConfig);
    HAL_Delay(1000);
   Set_Motor_Raw(0);

   HAL_Delay(2000);
   Enable_Motor();

#define IMU_DELAY 100
    if(HAL_UART_Transmit(&huart3, (uint8_t *)PASHR_ENABLE_CMD, 15, IMU_DELAY) != HAL_OK){
  	  printf("PASHR enable error \x0D\x0A");
    }

    //signal sent to set the transmission frequency for GPHDT sentence type
    if(HAL_UART_Transmit(&huart3, (uint8_t*)GPHDT_FREQ, 16, IMU_DELAY) != HAL_OK){
  	  printf("GPHDT frequency set error \x0D\x0A");
    }
  ecompass = NMEA0183__create(&huart3);

   // signal sent to initialize PASHR sentence type

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	while (NMEA0183__itemsInBuffer(ecompass) > 0) {
	  NMEA0183Raw *data = NMEA0183__getTopBufferItem(ecompass);
	  data->scentenceData[data->scentenceLength] = '\0';
	  //testing
	  printf("Message: %s", data->scentenceData);

	  printf("Data integrity test: %d\x0D\x0A", NMEA0183__checkMessage(data));
	  //		  printf("Message type: %s\x0D\x0A", NMEA0183__getField(data, 0));

	  if(NMEA0183__getScentenceType(data) == MESSAGE_SHR){

		//Get IMU data from message
	  	uint32_t heading = (atof(NMEA0183__getField(data, 2))+180)*100; //32 bits
	  	uint32_t pitch = (atof(NMEA0183__getField(data, 4))+180)*100; //32 bits
	  	uint32_t roll = (atof(NMEA0183__getField(data, 5))+180)*100; //32 bits

	  	//Update CAN frame
	  	rudder_debug_frame[2] = roll & 0xFF;
		rudder_debug_frame[3] = (((uint16_t) roll) >> 8) & 0xFF;
		rudder_debug_frame[4] = pitch & 0xFF;
		rudder_debug_frame[5] = (((uint16_t) pitch) >> 8) & 0xFF;
		rudder_debug_frame[6] = heading & 0xFF;
		rudder_debug_frame[7] = (((uint16_t) heading) >> 8) & 0xFF;

		//Update controller states
        controller.live.sailingState.currentHeading = heading / 100.0f;
        controller.live.sailingState.heelAngle = (roll / 100.0f) - 180.0f;

	  	printf("Euler Data: %lu, %lu, %lu \x0D\x0A", heading, roll, pitch);

	  	}
//Same data as above, easier for now to just do the one
	  //	  	else if(NMEA0183__getScentenceType(data) == MESSAGE_HDT){
//	  	  int8_t *heading_data = NMEA0183__getField(data, 1);
//
//	  	  uint32_t heading = (atof(heading_data)+180)*1000;
//	  	  printf("Heading Data: %u \x0D\x0A", heading);
//
//	  	  uint32_t euler[] = {heading};
//	  	}
	  	NMEA0183__incrementReadIndex(ecompass);
	  }
	  HAL_Delay(50);
//	  printf("here\r\n");

	#ifdef ZEROING_CALIBRATION
	  encoderZeroing(&huart1, encoderObject);
	#endif
//
	  if (controller_mode == 0)
		  runPID(&desiredRudderAngle);



    	  	//Update CAN frame
    uint16_t currentError = controller.live.liveValues.errorValue * 100;
    printf("current error: %f \r\n", controller.live.liveValues.errorValue);
    rudder_debug_frame[14] = currentError & 0xFF;
    rudder_debug_frame[15] = (((uint16_t) currentError) >> 8) & 0xFF;

    uint16_t currentDerivative = (controller.live.liveValues.derivativeValue + 300) * 100;
    rudder_debug_frame[12] = currentDerivative & 0xFF;
    printf("current derivative: %f \r\n", controller.live.liveValues.derivativeValue);
    rudder_debug_frame[13] = (((uint16_t) currentDerivative) >> 8) & 0xFF;

    uint16_t currentIntegral = controller.live.liveValues.integralValue + 30000;
    rudder_debug_frame[10] = currentIntegral & 0xFF;
    printf("current integral: %u \r\n", currentIntegral);
    rudder_debug_frame[11] = (((uint16_t) currentIntegral) >> 8) & 0xFF;

    uint16_t current_rudder_angle = (BRITER__floatAngle(encoderObject) + 90) * 100;
    rudder_debug_frame[0] = current_rudder_angle & 0xFF;
    rudder_debug_frame[1] = (((uint16_t) current_rudder_angle) >> 8) & 0xFF;
//    printf("Rudder angle: %f\r\n", BRITER__floatAngle(encoderObject));

    uint16_t commanded_rudder_angle = (desiredRudderAngle + 90) * 100;
//    printf("Commanded rudder angle: %hu\n desiredRudderAngle: %f\n", commanded_rudder_angle, desiredRudderAngle);
	rudder_debug_frame[8] = commanded_rudder_angle & 0xFF;
	rudder_debug_frame[9] = (((uint16_t) commanded_rudder_angle) >> 8) & 0xFF;

	  //Transmit CAN message after so long
	  // if (can_frame_tx_time + CAN_TX_DELAY_MS < HAL_GetTick()){
		//   TxHeader1.Identifier         = RUDDER_TO_MAINFRAME_DEBUG_ID;
		//   TxHeader1.IdType             = (RUDDER_TO_MAINFRAME_DEBUG_ID>0x7FF) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
		//   TxHeader1.TxFrameType        = FDCAN_DATA_FRAME;
		//   TxHeader1.DataLength         = byte_to_dlc(16);  /* HAL expects DLC in bits [19:16] */
		//   TxHeader1.ErrorStateIndicator= FDCAN_ESI_ACTIVE;
		//   TxHeader1.BitRateSwitch      = FDCAN_BRS_ON;
		//   TxHeader1.FDFormat           = FDCAN_FD_CAN;
		//   TxHeader1.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;

		//   if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader1,rudder_debug_frame)!=HAL_OK) {
		// 	printf("Err: CAN TX\r\n");
		//   }
	  // }

    if (can_frame_tx_time + CAN_TX_DELAY_MS < HAL_GetTick()){
      if(CAN_Transmit(RUDDER_TO_MAINFRAME_DEBUG_ID, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_16, rudder_debug_frame, &hfdcan1) != HAL_OK){
        Error_Handler();
      }
    }
	}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};
  DAC_AutonomousModeConfTypeDef sAutonomousMode = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Autonomous Mode
  */
  sAutonomousMode.AutonomousModeState = DAC_AUTONOMOUS_MODE_DISABLE;
  if (HAL_DACEx_SetConfigAutonomousMode(&hdac1, &sAutonomousMode) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
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
    HAL_NVIC_SetPriority(GPDMA1_Channel9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel9_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel15_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 31999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**UCPD1 GPIO Configuration
  PB15   ------> UCPD1_CC2
  PA15 (JTDI)   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBn_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_DBn_Pin */
  GPIO_InitStruct.Pin = UCPD_DBn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_DBn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	BRITER__handleDMA(encoderObject, huart, size);
	#ifndef ZEROING_CALIBRATION
		PI_Motor(desiredRudderAngle, BRITER__floatAngle(encoderObject), BRITER__getLastReadTimestamp(encoderObject));
	#endif
}

static uint8_t dlc_to_bytes(uint8_t dlc) {
    static const uint8_t dlc_lut[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
    };
    return dlc_lut[dlc & 0x0F];
}

static uint8_t byte_to_dlc(uint8_t len) {
	if (len <= 8) return len;
    else if (len == 12) return 9;
    else if (len == 16) return 10;
    else if (len == 20) return 11;
    else if (len == 24) return 12;
    else if (len == 32) return 13;
    else if (len == 48) return 14;
    else return 15;
}

uint32_t little_endian_bytes_to_uint(const uint8_t *bytes, uint8_t length) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < length; i++) {
        value |= ((uint32_t)bytes[i]) << (8 * i);
    }
    return value;
}

uint32_t little_endian_bytes_to_uint32(const uint8_t *bytes) {
    return little_endian_bytes_to_uint(bytes, 4);
}

uint16_t little_endian_bytes_to_uint16(const uint8_t *bytes) {
    return (uint16_t)little_endian_bytes_to_uint(bytes, 2);
}

uint8_t little_endian_bytes_to_uint8(const uint8_t *bytes) {
    return (uint8_t)little_endian_bytes_to_uint(bytes, 1);
}

void unpackGPSData(uint8_t * rxData) {
    uint32_t raw_speed = little_endian_bytes_to_uint32(&rxData[16]);
    controller.live.sailingState.linearVelocity = raw_speed / 3600; // speed in m/s
}

void unpackWindData(uint8_t * rxData) {
    uint16_t raw_wind_direction = little_endian_bytes_to_uint16(&rxData[0]);
    uint16_t raw_wind_speed = little_endian_bytes_to_uint16(&rxData[4]);
    controller.live.windState.windDirection = 360 - raw_wind_direction; // wind direction in degrees
    controller.live.windState.windSpeed = raw_wind_speed / 0.194384f; // wind speed in m/s
}

void unpackHeadingData(uint8_t * rxData) {
    uint32_t raw_heading = little_endian_bytes_to_uint32(&rxData[0]);
    printf("Raw heading: %lu \r\n", raw_heading);
    controller.live.sailingState.desiredHeading = 360-(((float) raw_heading) / 1000); // heading in degrees
    printf("Command heading: %f \r\n", controller.live.sailingState.desiredHeading);
}

void unpackCoefficients(uint8_t * rxData) {
    // Unpack PID coefficients from rxData and set them in controller settings
    uint32_t raw_kp = little_endian_bytes_to_uint32(&rxData[0]);
    uint32_t raw_ki = little_endian_bytes_to_uint32(&rxData[4]);
    uint32_t raw_kd = little_endian_bytes_to_uint32(&rxData[8]);

    controller.fixed.standardCoeffs.Kp = raw_kp / 1000000.0f;
    controller.fixed.standardCoeffs.Ki = raw_ki / 1000000.0f;
    controller.fixed.standardCoeffs.Kd = raw_kd / 1000000.0f;

    printf("KP: %f\r\n", controller.fixed.standardCoeffs.Kp);
    printf("KI: %f\r\n", controller.fixed.standardCoeffs.Ki);
	printf("KD: %f\r\n", controller.fixed.standardCoeffs.Kd);
}
void processCANFrames(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData) {
  // Need a switch based on rxHeader->Identifier
  // WITHIN EACH CASE, extract data from rxData and set variable in sailing state
	uint8_t length = dlc_to_bytes(RxHeader1.DataLength);
  switch(rxHeader->Identifier) {

    case 0x001:
        // Desired heading
    	if (length == 5){
    		if(RxData1[4] >> 7 == 1){
//    			printf("Manual Mode\r\n");
    			controller_mode = 1;
    			uint32_t rawSteeringCMD = little_endian_bytes_to_uint32(RxData1);
    			desiredRudderAngle = rawSteeringCMD / 1000.0f - 90;
    		} else {
//    			printf("Auto Mode\r\n");
    			if (controller_mode == 1){
    				controller.live.controllerState.lastTime = HAL_GetTick();
    				controller.live.controllerState.integralError = 0;
    				controller.live.controllerState.previousError = 0;
    			}
    			controller_mode = 0;
    			unpackHeadingData(rxData);
    		}
    	}

        break;

    // case 0x040: {
    //     // Wind data 1
    //     void unpackWindData(rxData) {
    // }

    case 0x041:
        // Wind data 2
    	if (length == 4)
    		unpackWindData(rxData);
        break;

//Won't get x070 frame this test
//    case 0x070:
//      //GPS Data
//      unpackGPSData(rxData);
//      break;

    case 0x200:
        // PID Coefficients
    	if (length == 12)
    		unpackCoefficients(rxData);
        break;

    default:
    	break;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	  {
    // Read message from RX FIFO 0
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, RxData1) != HAL_OK)
		{
			Error_Handler();
		}
	  }
     //Process the received message
    processCANFrames(&RxHeader1, RxData1);
}
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
  Disable_Motor();
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
