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
#define MAX_INPUT_BUFFER 18

//The timer prescalar value
#define PRESCALAR 1599

//The APB1 frequency in kHz
#define APB1_FREQUENCY 160000

//The constants to convert from BNO055 euler outputs to our standard degrees
#define EULER_CONSTANT 62.5

//The address used to check the calibration status of the system, gyroscope, accelerometer and magnetometer consecutively
#define BNO055_CalibStat 0x35

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- CONSTANTS -----------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//The register address of the heading data for the BNO055 (following 6 registers contain data for heading, roll and pitch consecutively)
const uint8_t headingRegisterAddress = 0x1A;

//The register address of the offset data for the BNO055 accelerometer X-axis LSB (following 18 registers contain data for accelerometer, magnetometer and gyroscope consecutively)
const uint8_t accX_LSB_RegisterAddress = 0x55;

//The register and value that needs to be sent for the IMU to leave CONFIG mode and enter IMU mode
//const uint8_t imuModeData[] = { 0x3D, 0x08 };
const uint8_t imuModeData[] = { 0x3D, 0x0C };

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//Initializes the IMU object
void IMU__init(IMU* self, I2C_HandleTypeDef* i2cChannel, TIM_HandleTypeDef* timChannel, uint8_t timeBetweenSamples, volatile uint32_t* headingOutput, volatile uint32_t* rollOutput, volatile uint32_t* pitchOutput, volatile uint32_t* accX_Offset, volatile uint32_t* accY_Offset, volatile uint32_t* accZ_Offset, volatile uint32_t* magX_Offset, volatile uint32_t* magY_Offset, volatile uint32_t* magZ_Offset, volatile uint32_t* gyrX_Offset, volatile uint32_t* gyrY_Offset, volatile uint32_t* gyrZ_Offset) {
//void IMU__init(IMU* self, I2C_HandleTypeDef* i2cChannel, TIM_HandleTypeDef* timChannel, uint8_t timeBetweenSamples, volatile uint32_t* headingOutput, volatile uint32_t* rollOutput, volatile uint32_t* pitchOutput) {
	//Copy the required data to the IMU object
	self->I2cHandle = i2cChannel;
	self->timHandle = timChannel;
	self->headingOutput = headingOutput;
	self->rollOutput = rollOutput;
	self->pitchOutput = pitchOutput;
	self->data_flag = 0;
	self->accX_Offset = accX_Offset;
	self->accY_Offset = accY_Offset;
	self->accZ_Offset = accZ_Offset;
	self->magX_Offset = magX_Offset;
	self->magY_Offset = magY_Offset;
	self->magZ_Offset = magZ_Offset;
	self->gyrX_Offset = gyrX_Offset;
	self->gyrY_Offset = gyrY_Offset;
	self->gyrZ_Offset = gyrZ_Offset;

	//Adjust the timer so that we get interrupts on the specified frequency
	__HAL_TIM_SET_PRESCALER(timChannel, PRESCALAR);
	__HAL_TIM_SET_AUTORELOAD(timChannel, APB1_FREQUENCY/(PRESCALAR+1)*timeBetweenSamples);
	__HAL_TIM_SET_COUNTER(timChannel, 0);

	//Transmit the instruction to get into IMU mode
	HAL_I2C_Master_Transmit(self->I2cHandle, BNO055_ADDR, (uint8_t *) imuModeData, 2, HAL_MAX_DELAY);
	HAL_Delay(10);

	//IMU_getOffset(IMU* self);

	//Can someone figure out why this shit hard faults when I put this here instead of in the main code
	//HAL_TIM_Base_Start_IT(timChannel);
	//HAL_Delay(timeBetweenSamples);
}

IMU* IMU__create(I2C_HandleTypeDef* i2cChannel, TIM_HandleTypeDef* timChannel, uint8_t timeBetweenSamples, volatile uint32_t* headingOutput,  volatile uint32_t* rollOutput, volatile uint32_t* pitchOutput, volatile uint32_t* accX_Offset, volatile uint32_t* accY_Offset, volatile uint32_t* accZ_Offset, volatile uint32_t* magX_Offset, volatile uint32_t* magY_Offset, volatile uint32_t* magZ_Offset, volatile uint32_t* gyrX_Offset, volatile uint32_t* gyrY_Offset, volatile uint32_t* gyrZ_Offset) {
//IMU* IMU__create(I2C_HandleTypeDef* i2cChannel, TIM_HandleTypeDef* timChannel, uint8_t timeBetweenSamples, volatile uint32_t* headingOutput,  volatile uint32_t* rollOutput, volatile uint32_t* pitchOutput){
	//Allocate memory for the IMU object and initialize it
	IMU* result = (IMU*)malloc(sizeof(IMU));
	result->inputBuffer = (uint8_t *) malloc(MAX_INPUT_BUFFER);
	result->calibStat = (uint8_t *) malloc(1);
	result->calibObject = (CALIBSTAT*) malloc(sizeof(CALIBSTAT));
	IMU__init(result, i2cChannel, timChannel, timeBetweenSamples, headingOutput, rollOutput, pitchOutput, accX_Offset, accY_Offset, accZ_Offset, magX_Offset, magY_Offset, magZ_Offset, gyrX_Offset, gyrY_Offset, gyrZ_Offset);
//	IMU__init(result, i2cChannel, timChannel, timeBetweenSamples, headingOutput, rollOutput, pitchOutput);
	return result;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- Calibration MANAGEMENT ----------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CALIBSTAT* BNO055_ReadCalibStat(IMU* self){
	//make this have an if statement for HAL status
//	if (self->calibStat == NULL) {
//	    printf("Error: calibStat is NULL, allocating memory...\r\n");
//	    self->calibStat = (uint8_t*) malloc(1);  // Allocate 1 byte
//	    if (self->calibStat == NULL) {
//	        printf("Error: Memory allocation failed!\r\n");
//	        return NULL;
//	    }
//	}

	if (HAL_I2C_Mem_Read(self->I2cHandle, BNO055_ADDR, BNO055_CalibStat, 1, self->calibStat, 1, HAL_MAX_DELAY) != HAL_OK){
//		printf("Error: failed to read calibration status \r\n");
	}
	else {
		//CALIBSTAT* calibObject;
		//returnVal = (CALIBSTAT*) malloc(sizeof(CALIBSTAT));

		//********this hard-faults whether its constants or bit-masking???????????????????????
		uint8_t calibVal = *(self->calibStat);
		self->calibObject->sysStat = (calibVal >> 6) & 0x03;
		self->calibObject->gyrStat = (calibVal >> 4) & 0x03;
		self->calibObject->accStat = (calibVal >> 2) & 0x03;
		self->calibObject->magStat = (calibVal) & 0x03;


		/*self->calibObject->sysStat = ((uint8_t) self->calibStat >> 6) & 0x03;
		self->calibObject->gyrStat = ((uint8_t) self->calibStat >> 4) & 0x03;
		self->calibObject->accStat = ((uint8_t) self->calibStat >> 2) & 0x03;
		self->calibObject->magStat = ((uint8_t) self->calibStat) & 0x03;
		 */

		/*self->calibObject->sysStat = self->calibStat[0];
		self->calibObject->sysStat = ((self->calibObject->sysStat << 8) | self->calibStat[1]);
		self->calibObject->gyrStat = self->calibStat[2];
		self->calibObject->gyrStat = ((self->calibObject->gyrStat << 6) | self->calibStat[3]);
		self->calibObject->accStat = self->calibStat[4];
		self->calibObject->accStat = ((self->calibObject->accStat << 4) | self->calibStat[5]);
		self->calibObject->magStat = self->calibStat[6];
		self->calibObject->magStat = ((self->calibObject->magStat << 2) | self->calibStat[7]);
		 */

		//self->calibObject->gyrStat = (calibStat >> 4) & 0x03;
		//self->calibObject->accStat = (calibStat >> 2) & 0x03;
		//self->calibObject->magStat = (calibStat) & 0x03;
		return self->calibObject;
	}
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- DATA PARSING FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void IMU__handleTxDMA(IMU* self, I2C_HandleTypeDef *I2cHandle){
	if(self->I2cHandle->Instance == I2cHandle->Instance){
		if(self->data_flag == 1){
			if(HAL_I2C_Master_Receive_DMA(I2cHandle, BNO055_ADDR, self->inputBuffer, 6) != HAL_OK){
//				printf("Error: failed to receive euler data \r\n");
			}
		}
		else if (self->data_flag == 2){
			if (HAL_I2C_Master_Receive_DMA(I2cHandle, BNO055_ADDR, self->inputBuffer, 18) != HAL_OK){
//				printf("Error: failed to receive offset data \r\n");
			}
		}
	}
}

void IMU__handleRxDMA(IMU* self, I2C_HandleTypeDef *I2cHandle){
	if(self->I2cHandle->Instance == I2cHandle->Instance){
		if(self->data_flag == 1){
			self->headingOutput[0] = (uint16_t)(self->inputBuffer[1] << 8 | self->inputBuffer[0]) * EULER_CONSTANT;
			self->rollOutput[0] = (int16_t)(self->inputBuffer[2] << 8 | self->inputBuffer[3]) * EULER_CONSTANT;
			self->pitchOutput[0] = (int16_t)(self->inputBuffer[4] << 8 | self->inputBuffer[5]) * EULER_CONSTANT;
		}
		else if(self->data_flag == 2){

			self->accX_Offset[0] = (self->inputBuffer[1] << 8 | self->inputBuffer[0]) * EULER_CONSTANT;
			self->accY_Offset[0] = (self->inputBuffer[2] << 8 | self->inputBuffer[3]) * EULER_CONSTANT;
			self->accZ_Offset[0] = (self->inputBuffer[4] << 8 | self->inputBuffer[5]) * EULER_CONSTANT;
			self->magX_Offset[0] = (self->inputBuffer[6] << 8 | self->inputBuffer[7]) * EULER_CONSTANT;
			self->magY_Offset[0] = (self->inputBuffer[8] << 8 | self->inputBuffer[9]) * EULER_CONSTANT;
			self->magZ_Offset[0] = (self->inputBuffer[10] << 8 | self->inputBuffer[11]) * EULER_CONSTANT;
			self->gyrX_Offset[0] = (self->inputBuffer[12] << 8 | self->inputBuffer[13]) * EULER_CONSTANT;
			self->gyrY_Offset[0] = (self->inputBuffer[14] << 8 | self->inputBuffer[15]) * EULER_CONSTANT;
			self->gyrZ_Offset[0] = (self->inputBuffer[16] << 8 | self->inputBuffer[17]) * EULER_CONSTANT;

//			uint8_t offsetReg = *(self->Offset);
//			offset->accX_Offset[0] = (self->inputBuffer[1] << 8 | self->inputBuffer[0]) * EULER_CONSTANT;
//			offset->accY_Offset[0] = (self->inputBuffer[2] << 8 | self->inputBuffer[3]) * EULER_CONSTANT;
//			offset->accZ_Offset[0] = (self->inputBuffer[4] << 8 | self->inputBuffer[5]) * EULER_CONSTANT;
//			offset->magX_Offset[0] = (self->inputBuffer[6] << 8 | self->inputBuffer[7]) * EULER_CONSTANT;
//			offset->magY_Offset[0] = (self->inputBuffer[8] << 8 | self->inputBuffer[9]) * EULER_CONSTANT;
//			offset->magZ_Offset[0] = (self->inputBuffer[10] << 8 | self->inputBuffer[11]) * EULER_CONSTANT;
//			offset->gyrX_Offset[0] = (self->inputBuffer[12] << 8 | self->inputBuffer[13]) * EULER_CONSTANT;
//			offset->gyrY_Offset[0] = (self->inputBuffer[14] << 8 | self->inputBuffer[15]) * EULER_CONSTANT;
//			offset->gyrZ_Offset[0] = (self->inputBuffer[16] << 8 | self->inputBuffer[17]) * EULER_CONSTANT;
		}
		self->data_flag = 0;
	}
}

void IMU__updateBuffer(IMU* self, TIM_HandleTypeDef* timChannel){
	if(self->timHandle->Instance == timChannel->Instance){
		if(self->data_flag == 0){
			if(HAL_I2C_Master_Transmit_DMA(self->I2cHandle, BNO055_ADDR, (uint8_t *) &headingRegisterAddress, 1) != HAL_OK){
//				printf("Error: failed to transmit signal for euler data \r\n");
			}
			else{
				self->data_flag = 1;
			}
		}
	}
}

void IMU_getOffset(IMU* self){
	if(self->data_flag == 0){
		if(HAL_I2C_Master_Transmit_DMA(self->I2cHandle, BNO055_ADDR, (uint8_t *) &accX_LSB_RegisterAddress, 1) != HAL_OK){
//			printf("Error: failed to transmit signal for offset data \r\n");
		}
		else{
			self->data_flag = 2;
		}
	}
}
