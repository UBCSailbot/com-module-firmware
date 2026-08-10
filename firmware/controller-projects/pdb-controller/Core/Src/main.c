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
#include <math.h>
#include <string.h>
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


// moving average struct
#define DELAY_LINE_SIZE 5
typedef struct {
    int16_t elements[DELAY_LINE_SIZE]; //circular buffer to store the most recent N readings
    uint8_t count;
    uint8_t  newestElement_idx;  //index of the newest element in the circular buffer
}delayLine;


typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
} NTC_Config;

typedef struct {
	const char*label;
	int is_temp;
} ADC_Channel_Info ;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_SENSORS 11
#define VREF 3.3
#define ADC_RESOLUTION 4095.0
#define R_FIXED 10000.0
#define BETA 3950.0
#define T0 298.15 // 25°C in Kelvin
#define R0 10000.0 // Resistance at 25°C
#define temp_threshold 55.0
#define voltage_threshold 2.5
#define CAN_TX_TIME 1000 //in milliseconds
#define PDB_HEARTBEAT 0x130
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
NTC_Config ntc_sensors[NUM_SENSORS] = {
	{&hadc1, ADC_CHANNEL_8},   // PA_3 = ADC1_IN8 ADC Cell 2
	{&hadc1, ADC_CHANNEL_7},   // PA2 = ADC1_IN7 TEMP_PACK2
	{&hadc1, ADC_CHANNEL_4},  // PC3 = ADC1_IN4 Cell 3 ADC
	{&hadc1, ADC_CHANNEL_15},  // PB0 = ADC1_IN15 TEMP_PACKB/B
	{&hadc1, ADC_CHANNEL_2},  // PC1 = ADC1_IN2 TEMP_PACK 1 (T7 shares PC0 with T5, duplicate?)
	{&hadc1, ADC_CHANNEL_1},  //  PC0 = ADC1_IN1 ADC Cell 4
	{&hadc1, ADC_CHANNEL_17},  // PB_2 ADC1_IN17 ADC Cell 1
	{&hadc1, ADC_CHANNEL_6},   // PA1 = ADC1_IN2
	{&hadc1, ADC_CHANNEL_5},   // PA0 = ADC1_IN1
	{&hadc1, ADC_CHANNEL_16},   // PB1 = ADC1_IN16
	{&hadc4, ADC_CHANNEL_7},   // PG0 = ADC3_IN8 (ADC4)
};

ADC_Channel_Info channel_info[7] = {
    {"VC2_Cumulative",0},
    {"TempPack2",1},
    {"VC3_Cumulative",0},
    {"TempPack B/B",1},
    {"TempPack1",1},
    {"VC4_Cumulative",0},
    {"VC1_Cumulative",0},
};

volatile int restart_requested = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC4_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

// moving avaerage function prototypes
void delayLine_Init(delayLine *dl);
void delayLine_addElement(delayLine *dl, int16_t newValue);
int16_t delayLine_MovingAverage(delayLine *dl, int32_t *accumulator, int16_t x);




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

static float ADC_Select_Channel(uint32_t channelNumber)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t raw;

  /**>RRE Configure ADC1 Channel */
  sConfig.Channel = (uint32_t)ntc_sensors[channelNumber - 1].channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_814CYCLES ;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(ntc_sensors[channelNumber - 1].hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /**>RRE Perform Conversion */
  HAL_ADC_Start(ntc_sensors[channelNumber - 1].hadc);
  HAL_ADC_PollForConversion(ntc_sensors[channelNumber - 1].hadc, 1000);
  raw = HAL_ADC_GetValue(ntc_sensors[channelNumber - 1].hadc);
  HAL_ADC_Stop(ntc_sensors[channelNumber - 1].hadc);

      float v = (raw / ADC_RESOLUTION) * VREF;
      float r_ntc = (v * R_FIXED) / (VREF - v);
      float tempK = 1.0 / ((log(r_ntc / R0) / BETA) + (1.0 / T0));

      //printf("Raw: %lu, V: %.2fV, R_NTC: %.1fΩ, Temp: %.2f°C\r\n", raw, v, r_ntc, tempK);
      //return tempK - 273.15;
      if ((channelNumber == 2)||(channelNumber == 4)||(channelNumber == 5)) return tempK - 273.15; // 2 and 4 are TEMP values not ADC values
      else return v;
}


// moving average functions
void delayLine_Init(delayLine *dl)
{
    dl->newestElement_idx = 0;
    dl->count = 0;

    for (int i = 0; i < DELAY_LINE_SIZE; i++) {
        dl->elements[i] = 0;
    }
}

/*
 1-Find the slot that will be overwritten.
 2-Subtract its old value from the sum.
 3-Write the new value into that same slot.
 */


void delayLine_addElement(delayLine *dl, int16_t newValue)
{
    dl->newestElement_idx++;
    dl->newestElement_idx = dl->newestElement_idx % (DELAY_LINE_SIZE);
    dl->elements[dl->newestElement_idx] = newValue;
}

int16_t delayLine_MovingAverage(delayLine *dl, int32_t *accumulator, int16_t x)
{
    int oldest_idx = (dl->newestElement_idx + 1) % (DELAY_LINE_SIZE); // overflow check

    if (dl->count == DELAY_LINE_SIZE) {
        *accumulator -= dl->elements[oldest_idx];
    } else {
        dl->count++;
    }

    *accumulator += x;
    delayLine_addElement(dl, x);

    return *accumulator / dl->count;
}

/*
 * CUSTOM CAN FUNCTIONS
 */
// only thing different is the filters, need access to FIFO1 callback for one std ID

void CAN_Init_PDB(FDCAN_HandleTypeDef *hfdcan1)
{
	// Set frame 0x202 to be filtered to FIFO1
    FDCAN_FilterTypeDef f = {0};
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;      // exact match using mask
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    f.FilterID1    = 0x202;                  // Only can frame sent to PDB
    f.FilterID2    = 0x7FF;                  // mask: match all 11 bits exactly

    if (HAL_FDCAN_ConfigFilter(hfdcan1, &f) != HAL_OK)
    {
        Error_Handler();
    }
    // Set extended IDs to be rejected (unused)
    FDCAN_FilterTypeDef fe = {0};
    fe.IdType       = FDCAN_EXTENDED_ID;
    fe.FilterIndex  = 0;
    fe.FilterType   = FDCAN_FILTER_RANGE_NO_EIDM;
    fe.FilterConfig = FDCAN_FILTER_REJECT;
    fe.FilterID1    = 0x00000000;
    fe.FilterID2    = 0x1FFFFFFF;

    if (HAL_FDCAN_ConfigFilter(hfdcan1, &fe) != HAL_OK)
    {
        Error_Handler();
    }
    /* Global filter: send non-matching frames to FIFO0 (same as global can lib) */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    // Start FDCAN
    if (HAL_FDCAN_Start(hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * Custom FIFO1 Callback
 */
// immediately handles shutdown/restart commands
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U)
        return;

    FDCAN_RxHeaderTypeDef rxh;
    uint8_t tmp[64];
    memset(tmp, 0, sizeof(tmp));

    uint8_t power_off = 0xF;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxh, tmp) != HAL_OK) {
        Error_Handler();
    }

    // theoretically because of the filter, FIFO1 should only contain STD 0x202 (but check anyway)
    if ((rxh.IdType == FDCAN_STANDARD_ID) && (rxh.Identifier == 0x202))
    {
    	if (tmp[0] == 0x0A) // shutdown
    	{
    		// printf("Shutdown command was received\r\n");, removed cuz i feel like we doin to much in isr
    		// to frame 0x003, 0xF -> shutdown

    		// power off everything
    		if( CAN_Transmit( 0x003, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_8, &power_off, hfdcan) != HAL_OK ) {
    			Error_Handler();
    		}

    		//set PC_8 to low
    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //PC_8 is the boot pin (inverted logic)
    	    // printf("PC_8 Pulled Low \r\n");
    	}
    	else if (tmp[0] == 0x14) // restart after 20s
    	{
    		// printf("restart command was received\r\n");
    		restart_requested = 1;
    	}

    }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    printf("Tx buffer complete. BufferIndexes: 0x%lX\r\n", BufferIndexes);
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
  MX_ADC1_Init();
  MX_ICACHE_Init();
  MX_UCPD1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC4_Init();
  MX_FDCAN1_Init();
  MX_I2C2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	  delayLine adc_filters[7];
	  int32_t adc_accumulators[7] = {0};

	  for (int i = 0; i < 7; i++) {
		  delayLine_Init(&adc_filters[i]);
	  }
	  /*
	   * I2C Variable Declarations & ADC Initialization
	   */

	  uint8_t I2C_buf[12];

	  // Address of ADC device on current sense board
	  uint16_t ADC_ADDR1 = 0x18 << 1; // 0x18, if J1 disconnected - STARBOARD
	  uint16_t ADC_ADDR2 = 0x1F << 1; // 0x1F, if J1 connected - PORT

	  I2C_buf[0] = 0x08; //opcode for single register write
	  // I2C MODE setup
	  I2C_buf[1] = 0x1c; // MODE SELECT register address
	  I2C_buf[2] = 0x04; // selecting MANUAL MODE W/ AUTO SEQUENCING


//	  if( HAL_I2C_Master_Transmit(&hi2c2,ADC_ADDR1,I2C_buf,3,HAL_MAX_DELAY) != HAL_OK ) {
//		  Error_Handler();
//	  }
//	  if( HAL_I2C_Master_Transmit(&hi2c2,ADC_ADDR2,I2C_buf,3,HAL_MAX_DELAY) != HAL_OK ) {
//		  Error_Handler();
//	  }

	  // I2C START SEQUENCE Bit
	  I2C_buf[1] = 0x1E; // START SEQ register
	  I2C_buf[2] = 0b1; // bit starts first conversion


//	  if( HAL_I2C_Master_Transmit(&hi2c2,ADC_ADDR1,I2C_buf,3,HAL_MAX_DELAY) != HAL_OK ) {
//		  Error_Handler();
//	  }
//	  if( HAL_I2C_Master_Transmit(&hi2c2,ADC_ADDR2,I2C_buf,3,HAL_MAX_DELAY) != HAL_OK ) {
//		  Error_Handler();
//	  }

	/*
	 * CAN Initialization
	 */
	CAN_Init(&hfdcan1, PDB_HEARTBEAT);
	if( HAL_FDCAN_Stop(&hfdcan1) != HAL_OK ) {
		Error_Handler();
	}
  	CAN_Init_PDB(&hfdcan1); // call custom init code w diff filters

  	/*
  	 * Temperature Sense Variable Initialization
  	 */
  	int count = 0;
  	float adc_readings[7] = {0}; //store all adc readings here
  	float vc[4] = {0};
  	int vc_index=0 ;
  	uint16_t individual_voltages[4] = {0};
  	int temp_back = 1 ; //flag to break the infinite loop in case we are back on
  	int voltage_back = 1; //flag to break the infinite loop in case we are back on
  	float cell_voltage[4] = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  vc_index = 0; // reset the value of the index

	  HAL_Delay(100);//0.1 SECONDS AS OF NOW

	  /*
	   * Boot pin logic
	   */
	  if (restart_requested) {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //turn pin off (inverted logic)
		  printf("PC_8 Pulled Low\r\n");
	      for (int sec = 20; sec > 0; sec--) {
	          printf("Restarting in %d seconds...\r\n", sec);
	          HAL_Delay(1000);
	      }

	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //turn pin back on (inverted logic)
	      printf("PC_8 Pulled High\r\n");

	      restart_requested = 0;
	  }

	  //comment out the transmission to figure out how to receive
	  // CHANGED TO INITIALIZE TO 0s
	  uint8_t TxData1[24] = {0}; //the array that we use to send CAN
	  int byte_index = 0;

	  /*
	   * ADC For-loop Start
	   */
	  for(int i = 1; i <= 7; i++) {
		  // read the raw adc value
		  float adc_raw = ADC_Select_Channel(i);

		  int16_t adc_scaled;

		  if (channel_info[i - 1].is_temp) {
		      adc_scaled = (int16_t)(adc_raw * 100);
		  }
		  else {
		      adc_scaled = (int16_t)(adc_raw * 1000);
		  }

		  // calculate the moving average
		  int16_t adc_avg_scaled = delayLine_MovingAverage(
		      &adc_filters[i - 1],
		      &adc_accumulators[i - 1],
		      adc_scaled
		  );

		  float adc;

		  if (channel_info[i - 1].is_temp) {
		      adc = adc_avg_scaled / 100.0f;
		  }
		  else {
		      adc = adc_avg_scaled / 1000.0f;
		  }

		  ADC_Channel_Info info = channel_info[i-1];
		  adc_readings[i-1] = adc; // store the ADC value

		  int16_t encoded_val;

		  // Check Temperature Reading
		  if (info.is_temp) {
			  encoded_val = (int16_t)(adc * 100);  //to get rid of the decimal places (this gives us 4 hex digits)

			  // If temp is too high, shut off system and poll until regular conditions, then reboot
			  if (adc >= temp_threshold ) {
				  printf("%s temperature too high! %.2f°C CANT: %d CANT: %X \r\n", info.label, adc ,encoded_val , encoded_val);

				  // toggle pin PC_8, shuts off the entire system
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //PC_8 is the boot pin
		          temp_back = 0;

		          while (temp_back == 0 ) {
		        	  for (int y=20 ; y > 0 ; y-- ) {
		        		  printf("checking the temp in %d seconds" , y);
		        		  HAL_Delay(1000);
		        	  }

		        	  // read the pin again
		        	  adc = ADC_Select_Channel(i);
		        	  ADC_Channel_Info info = channel_info[i-1];
		        	  adc_readings[i-1] = adc; // store the adc value

		        	  if (adc <= temp_threshold ) {
		        	  		temp_back = 1; // break out of loop and continue
		        	  		printf("%s is below threshold \r\n", info.label);
		        	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //PC_8 power back on
		        	  }
		          }

		      }
			  else { // temperature normal
				  printf("%s: %.2f°C CANT: %d \r\n", info.label, adc , encoded_val );
		      }

		  }
		  // Check voltage (not temp) reading
		  else {
			  encoded_val = (int16_t)(adc*1000*5);  // scaling to get 4 clean hex digits

			  // voltage reading too low
			  if (adc*5 <= voltage_threshold ) {
				  printf("%s below the recommended range! %.2f CANV: %d CANV: %X \r\n", info.label , 5*adc, encoded_val, encoded_val);
		        }
			  // regular voltage reading
		      else {
		            printf("%s: %.2fV CANV: %d CANV: %X\r\n", info.label, 5*adc ,encoded_val, encoded_val);
		      }
		  }

		  /*
		   * Build VC array
		   */

		  // Build vc[] array only for your voltage channels (1,3,6,7)
		  if ((i == 1) || (i == 3) || (i == 6) || (i == 7)) {
			  vc[vc_index++] = adc * 5;
		  }

		  // When you reach the last channel, compute individual voltages and print
		  if (i == 7) {
		               for (int j = 0; j < 4; j++) {
		                   printf("%.2f ", vc[j]);
		               }
		               printf("\r\n");

		               cell_voltage[1] = vc[0] - vc[3]; //cell 2
		               cell_voltage[2] = vc[1] - vc[0]; //cell 3
		               cell_voltage[3] = vc[2] - vc[1]; //cell 4
		               cell_voltage[0] = vc[3]; //cell 1


		               individual_voltages[0] = (vc[0] - vc[3]) * 1000;
		               individual_voltages[1] = (vc[1] - vc[0]) * 1000;
		               individual_voltages[2] = (vc[2] - vc[1]) * 1000;
		               individual_voltages[3] = (vc[3]) * 1000;

		               printf ("Cell1:%0.2f Cell2:%0.2f Cell3:%0.2f Cell4:%0.2f \r\n", cell_voltage[0] , cell_voltage[1] , cell_voltage[2] , cell_voltage[3] );

		               for (int k = 0; k < 4; k++) {

		            	   // Cell voltage too low
		            	   if (cell_voltage[k] <= voltage_threshold ) {
								// toggle pin PC_8 it shuts off the entire system
								printf("Cell %d voltage is below threshold %0.2f \r\n" , (k+1) , cell_voltage[k]);

								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //PC_8 is the boot pin

								voltage_back = 0;
								// wait 20 seconds and enter an infinite loop of saying the batteries are under the
								//operating voltage and check if you are back on
		            	   }
		            	   
		            	   // Keep reading/computing cell voltage until back to normal
		            	   while(voltage_back == 0) {
		            	       // Wait 20 seconds
		            	       for (int o = 20; o > 0; o--) {
		            	           printf("checking status of cell%d in %d seconds again\r\n", k+1 , o);
		            	           HAL_Delay(1000);
		            	       }

		            	       vc_index = 0; // reset before refilling

		            	       // Re-read all 4 voltage channels: i = 1, 3, 6, 7
		            	       int voltage_channels[] = {1, 3, 6, 7};

		            	       for (int ch = 0; ch < 4; ch++) {
		            	           float refreshed_adc = ADC_Select_Channel(voltage_channels[ch]);
		            	           vc[vc_index++] = refreshed_adc * 5;
		            	       }

		            	       // Recompute cell voltages
		            	       cell_voltage[1] = vc[0] - vc[3]; //cell 2
		            	       cell_voltage[2] = vc[1] - vc[0]; //cell 3
		            	       cell_voltage[3] = vc[2] - vc[1]; //cell 4
		            	       cell_voltage[0] = vc[3];         //cell 1

		            	       // Update individual voltages
		            	       individual_voltages[0] = (vc[0] - vc[3]) * 1000;
		            	       individual_voltages[1] = (vc[1] - vc[0]) * 1000;
		            	       individual_voltages[2] = (vc[2] - vc[1]) * 1000;
		            	       individual_voltages[3] = vc[3] * 1000;

		            	       // Re-check threshold for that specific cell
		            	       if (cell_voltage[k] >= voltage_threshold) {
		            	           voltage_back = 1;
		            	           HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //turn pin back on
		            	           printf("Cell %d is back above threshold: %.2f\r\n", k+1, cell_voltage[k]);
		            	       }

		            	   }

		            	   for (int z=0 ; z<4 ; z++) {
		            		   printf("%f ", cell_voltage[z]);
		            	   }

		            	   printf("\r\n");
		               }
		  }
	  }

	  /*
	   * ADC For-loop end
	   */


	  /*
	   * Pack temp_values and individual_voltages into CAN Tx Buffer (TxData1)
	   */
	  int volt_index = 0;

	  for (int i = 1; i <= 7; i++) {
		  // encode values
		  int16_t encoded_val;

		  if (channel_info[i-1].is_temp) {
			  encoded_val = (int16_t)(adc_readings[i-1] * 100);
		  }
		  else {
              encoded_val = individual_voltages[volt_index++];
		  }

		  // Pack the encoded values into TxData1 (low byte first)
		  TxData1[byte_index++] = encoded_val & 0xFF;
		  TxData1[byte_index++] = (encoded_val >> 8) & 0xFF;
	  }

	  // Add two extra zero bytes
	  TxData1[byte_index++] = 0x00;
	  TxData1[byte_index++] = 0x00;

	  // Print TxData1 array
	  printf("\nTxData1 Array:\r\n");
	  for (int i = 0; i < 16; i++) {
		  printf("%X ", TxData1[i]);
	  }
	  printf("\r\n");

	  /*
	   * Current Sense I2C Receive & Calculations
	   */
	  // MPPT Board 1
	  //HAL_I2C_Master_Receive(&hi2c2,ADC_ADDR1,I2C_buf,4,HAL_MAX_DELAY); //reads 4 bytes of raw voltage data, 2 bytes from each mppt channel

	  // i2c buffer -> raw adc values from each mppt
	  uint16_t raw1 = ((uint16_t)I2C_buf[0] << 8 ) | I2C_buf[1];
	  uint16_t raw2 = ((uint16_t)I2C_buf[2] << 8 ) | I2C_buf[3];

	  // raw adc values -> original current values (x1000)
	  // conversion eq: (raw/2^16 *ref_V - offset) / scale * 1000
	  int16_t curr1 = (int16_t) ((raw1/65536.0f*3.3f - 0.5f)/0.2f*1000);
	  int16_t curr2 = (int16_t) ((raw2/65536.0f*3.3f - 0.5f)/0.2f*1000);

	  if ( curr1 < 0 ) curr1 = 0;
	  if ( curr2 < 0 ) curr2 = 0;

	  //store in Tx buffer (little endian)
	  TxData1[15] = ( curr1 >> 8 ) & 0x00FF; // MPPT 1_A
	  TxData1[14] = curr1 & 0x00FF;

	  TxData1[17] = ( curr2 >> 8 ) & 0x00FF; // MPPT 1_B
	  TxData1[16] = curr2 & 0x00FF;

	  // print to UART (just for debugging)
	  printf( "MPPT BOARD 1\n" );
	  printf("MPPT_1: I*1000  | CH0: %d, CH1: %d\r\n", curr1, curr2); // current * 1000
	  printf("MPPT_1: TxData  | %X_%X_%X_%X\r\n", TxData1[14], TxData1[15], TxData1[16], TxData1[17]); // tx buffer, exactly as it is sent

	  // MPPT Board 2
	  // same as above, just to different address & bytes

	  //HAL_I2C_Master_Receive(&hi2c2,ADC_ADDR2,I2C_buf,4,HAL_MAX_DELAY);

	  raw1 = ((uint16_t)I2C_buf[0] << 8 ) | I2C_buf[1];
	  raw2 = ((uint16_t)I2C_buf[2] << 8 ) | I2C_buf[3];

	  curr1 = (int16_t) ((raw1/65536.0f*3.3f - 0.5f)/0.2f*1000);
	  curr2 = (int16_t) ((raw2/65536.0f*3.3f - 0.5f)/0.2f*1000);

	  if ( curr1 < 0 ) curr1 = 0;
	  if ( curr2 < 0 ) curr2 = 0;

	  TxData1[19] = ( curr1 >> 8 ) & 0x00FF; // MPPT 2_A
	  TxData1[18] = curr1 & 0x00FF;

	  TxData1[21] = ( curr2 >> 8 ) & 0x00FF; // MPPT 2_B
	  TxData1[20] = curr2 & 0x00FF;

	  printf( "MPPT BOARD 2\n" );
	  printf("MPPT_2: I*1000  | CH0: %d, CH1: %d\r\n", curr1, curr2);
	  printf("MPPT_2: TxData  | %X_%X_%X_%X\r\n", TxData1[18], TxData1[19], TxData1[20], TxData1[21]);


	  /*
	   * Final CAN Transmission to RPI
	   */
	 if (CAN_Transmit(0x206, FDCAN_STANDARD_ID, FDCAN_DLC_BYTES_24, TxData1, &hfdcan1) != HAL_OK)
	 {
		 printf("HAL ERROR CODE: %lu\r\n", HAL_FDCAN_GetError(&hfdcan1));
		 printf("TS: %lu\r\n", HAL_GetTick());
		 printf("count: %d \r\n",count);
		 //Error_Handler();
	 }

	//  HAL_Delay(500); //changed this from 5000ms to this should be tested

	 count++;
	 printf("count: %d \r\n",count);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
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
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoPowerOff = ADC_LOW_POWER_NONE;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.SamplingTimeCommon1 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.SamplingTimeCommon2 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC4_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

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
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_GREEN_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBn_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
  NVIC_SystemReset();
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
