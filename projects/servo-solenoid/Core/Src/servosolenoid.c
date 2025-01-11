/*
 * servosolenoid.c
 *
 * Author: Faaiq Majeed
 * Date Last Updated: January 1, 2024
 * Description: Functions for moving the wingsail's servo motor and retracting and extending the solenoid. Also initialization of relevant timers and variables.
 */

/*
 * Includes:
 */

#include "servosolenoid.h"

/*
 * Variables:
 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/*
 * Functions:
 */

void initializeServoSolenoid(void)
{
	//Initializes the peripherals required for the Servo motor and Solenoid

	////////////////////////////////////////////
	/*       Configure the Servo Motor		  */
	////////////////////////////////////////////

	MX_TIM2_Init();
	MX_TIM7_Init();
    MX_TIM6_Init();


	HAL_TIM_Base_Stop_IT(&htim6);
	HAL_TIM_Base_Stop_IT(&htim7);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
	__HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);

	////////////////////////////////////////////
	/*Configure GPIO pin : SolenoidOutput_Pin */
	////////////////////////////////////////////

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SolenoidOutput_GPIO_Port, SolenoidOutput_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = SolenoidOutput_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SolenoidOutput_GPIO_Port, &GPIO_InitStruct);

}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2472;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65534;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2472;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
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
}

void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}


int is_TIM6_Running(void)
{
	//returns TRUE (1) if TIM6 is running and FALSE (0) if TIM6 is not running
	if((TIM6->CR1 & TIM_CR1_CEN) != 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int is_TIM7_Running(void)
{
	//returns TRUE (1) if TIM6 is running and FALSE (0) if TIM7 is not running
	if((TIM7->CR1 & TIM_CR1_CEN) != 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void moveServo(double angle)
{

	/***********************************************************************************************************************************
	* IMPORTANT NOTE:																												   *
	*																																   *
	* "angle" Parameter -> 0 =< angle <= 180																						   *
	* FREQUENCY = 50 HZ (DOUBLE CHECK THIS!!)																						   *					   				   *
	* CLOCK SPEED ON TIM 2 = 160 MHZ																							       *
	* PRE-SCALOR = 160-1 -> NEW CLOCK SPEED = 1 MHz																					   *
	* COUNTER-PERIOD = 5000-1 -> PWM FREQUENCY = 50 HZ = 20ms Time Period (CHECK WITH MICHAEL ON FREQUENCY/TIME PERIOD OF SERVO MOTOR) *
	* 																																   *
	* -90 Degrees =  2.5% Duty Cycle (DutyCycleInput == 500) -> absoluteAngle == 0;							   						   *
	* 0 Degrees = 7.5% Duty Cycle (DutyCycleInput == 1500)	-> absoluetAngle == 90				   			   						   *
	* 90 Degrees = 12.5% Duty Cycle (DutyCycleInput == 2500) -> absoluteAngle == 180						   						   *
	* 																																   *
	* GPIOE PIN 0 == SOLENOID PIN																									   *
	* 																																   *
	* (TIM6->CR1 && TIM_CR1_CEN) == 0 (Timer 6 is not running)																	       *
	* (TIM6->CR1 && TIM_CR1_CEN) != 0 (Timer 6 is running)																			   *
	* - Same Logic with TIM7																										   *
	* 																																   *
	* __HAL_TIM_SET_COMPARE Sets the Duty Cycle of the Servo (But does not turn it on or off)										   *
	* 																																   *
	* 											    													   							   *
	***********************************************************************************************************************************/

	uint32_t absoluteAngle = (uint32_t)(angle);

	//Setting Servo Duty Cycle
	uint32_t DutyCycleInput = 0;
	DutyCycleInput = (560) + (absoluteAngle * (2500 - 500) / 180);

	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == 0 && is_TIM6_Running() == 0 && is_TIM7_Running() == 0)
	{
		//Set Servo Duty Cycle
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, DutyCycleInput/10); //ARR = 20000

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);
		HAL_TIM_Base_Start_IT(&htim7);

	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == 1 && (is_TIM6_Running() == 0 && is_TIM7_Running() == 1))
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, DutyCycleInput); //ARR = 20000
	}
	else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == 1 && (is_TIM6_Running() == 1 && is_TIM7_Running() == 0))
	{
		HAL_TIM_Base_Stop_IT(&htim6);
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, DutyCycleInput); //ARR = 20000
		HAL_TIM_Base_Start_IT(&htim6);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   // Handle TIM7 interrupt here as well if needed
	if (htim->Instance == TIM7)
	{
		__HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
		// Stop TIM7
		HAL_TIM_Base_Stop_IT(&htim7);
		__HAL_TIM_SET_COUNTER(&htim7, 0);

		// Start TIM6 && Start Servo PWM Signal
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_TIM_Base_Start_IT(&htim6);

	}

    // Check if the interrupt is from TIM6
	else if (htim->Instance == TIM6)
    {
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        // Stop TIM6
        HAL_TIM_Base_Stop_IT(&htim6);
        __HAL_TIM_SET_COUNTER(&htim6, 0);
        //Stop Servo PWM Signal
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0); //Forces the PWM Output Pin to a 0V state (High Impedance).

        //Push Solenoid back in
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);

    }
}
