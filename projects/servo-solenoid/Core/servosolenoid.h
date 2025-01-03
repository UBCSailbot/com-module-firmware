/*
 * servosolenoid.h
 *
 * Author: Faaiq Majeed
 * Date Last Updated: January 1, 2024
 * Description: Header file for moving the wingsail's servo motor and retracting and extending the solenoid. Also initialization of relevant timers and variables.
 */

#ifndef SERVOSOLENOID_H
#define SERVOSOLENOID_H

/*
 * Includes:
 */
#include "main.h"

/*
 * Variables
 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;


/*
 * Functions
 */

void initializeServoSolenoid(void);

static void MX_TIM2_Init(void);

static void MX_TIM6_Init(void);

static void MX_TIM7_Init(void);

void TIM6_IRQHandler(void);

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void);

int is_TIM6_Running(void);

int is_TIM7_Running(void);

void moveServo(double angle);

//Used for timer interrupt handling
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
