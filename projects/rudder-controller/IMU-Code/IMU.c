/*
 * IMU.c
 *
 *  Created on: Dec 02, 2025
 *      Author: Heidi Leung, Michael Greenough
 */

#include "IMU.h"
#include <stdlib.h>
#include <stdio.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PRIVATE MACROS ------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//The address used for the IMU (0x28 is default) left shift by one
#define BNO055_ADDR 0x28 << 1

//The maximum number of sequential register bytes that can be read:
#define MAX_INPUT_BUFFER 2

//The timer prescalar value
#define PRESCALAR 1599

//The APB1 frequency in kHz
#define APB1_FREQUENCY 160000

//The constant to convert from BNO055 heading output to our standard
#define HEADING_CONSTANT 62.5

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- CONSTANTS -----------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//The register address of the heading data for the BNO055
const uint8_t headingRegisterAddress = 0x1A;

//The register and value that needs to be sent for the IMU to leave CONFIG mode and enter IMU mode
const uint8_t imuModeData[] = { 0x3D, 0x08 };

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//Initializes the IMU object
void IMU__init(IMU* self, I2C_HandleTypeDef* i2cChannel, TIM_HandleTypeDef* timChannel, uint8_t timeBetweenSamples, volatile uint32_t* headingOutput) {

	//Copy the required data to the IMU object
	self->I2cHandle = i2cChannel;
	self->timHandle = timChannel;
	self->headingOutput = headingOutput;

	//Adjust the timer so that we get interrupts on the specified frequency
	__HAL_TIM_SET_PRESCALER(timChannel, PRESCALAR);
	__HAL_TIM_SET_AUTORELOAD(timChannel, APB1_FREQUENCY/(PRESCALAR+1)*timeBetweenSamples);

	//Transmit the instruction to get into IMU mode
	HAL_I2C_Master_Transmit(self->I2cHandle, BNO055_ADDR, (uint8_t *) imuModeData, 2, HAL_MAX_DELAY);
	HAL_Delay(10);

	//Can someone figure out why this shit hard faults when I put this here instead of in the main code
	//HAL_TIM_Base_Start_IT(timChannel);
	//HAL_Delay(timeBetweenSamples);
}

IMU* IMU__create(I2C_HandleTypeDef* i2cChannel, TIM_HandleTypeDef* timChannel, uint8_t timeBetweenSamples, volatile uint32_t* headingOutput) {
	//Allocate memory for the IMU object and intialize it
	IMU* result = (IMU*)malloc(sizeof(IMU));
	result->inputBuffer = (uint8_t *) malloc(MAX_INPUT_BUFFER);
	IMU__init(result, i2cChannel, timChannel, timeBetweenSamples, headingOutput);
	return result;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- DATA PARSING FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void IMU__handleTxDMA(IMU* self, I2C_HandleTypeDef *I2cHandle){
	if(self->I2cHandle->Instance == I2cHandle->Instance){
		HAL_I2C_Master_Receive_DMA(I2cHandle, BNO055_ADDR, self->inputBuffer, 2);
	}
}

void IMU__handleRxDMA(IMU* self, I2C_HandleTypeDef *I2cHandle){
	if(self->I2cHandle->Instance == I2cHandle->Instance){
		self->headingOutput[0] = (self->inputBuffer[1] << 8 | self->inputBuffer[0]) * HEADING_CONSTANT;
	}
}

void IMU__updateBuffer(IMU* self, TIM_HandleTypeDef* timChannel){
	if(self->timHandle->Instance == timChannel->Instance){
		HAL_I2C_Master_Transmit_DMA(self->I2cHandle, BNO055_ADDR, (uint8_t *) &headingRegisterAddress, 1);
	}
}
