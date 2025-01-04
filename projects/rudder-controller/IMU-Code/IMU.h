/*
 * This library prrovides an interface to communicate between the microcontroller and a BNO055 IMU.
 * It currently has the following functionality:
 *		-Initialize the IMU for use
 *		-Read the heading
 *
 * For this library to work as intended:
 * 		"IMU__handleTxDMA()" must be called inside "HAL_I2C_MasterTxCpltCallback()"
 * 		"IMU__handleRxDMA()" must be called inside "HAL_I2C_MasterRxCpltCallback()"
 * 		"IMU__updateBuffer()" must be called inside "HAL_TIM_PeriodElapsedCallback()"
 * 		"HAL_TIM_Base_Start_IT()" must be called imediatly after creating the IMU object with the
 * 		timer channel handle pointer used in the creation of the IMU object as a parameter.
 * 		More information on the intended use can be found in the associated readme file.
 *
 *
 *  Created on: Dec 02, 2025
 *      Author: Heidi Leung, Michael Greenough
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_



 //----------------------------------------------------------------------------------------------------------------------------------------------------------------
 //--------------------------------------------------------------------------- INCLUDES ---------------------------------------------------------------------------
 //----------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "stm32u5xx_hal.h"

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

//The IMU data type
typedef struct {
	I2C_HandleTypeDef* I2cHandle;
	TIM_HandleTypeDef* timHandle;
	volatile uint32_t * headingOutput;
	uint8_t * inputBuffer;
} IMU;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Creates a new IMU object.
 *
 * @param i2cChannel The I2C Handle of the I2C peripheral to be used. Must be set up to be compatible with the BNO055.
 * @param timChannel The timer channel to be used with this peripheral. This is used to create an interrupt
 * 			for the periodic calling for data from the sensor.
 * @param timeBetweenSamples The time in ms between sample collection from the sensor.
 * @param headingOutput Points to a location in memory where the heading is automatically updated. It is in the format
 * 			of heading * 1000 in degrees where 0 is north and increasing in the CW direciton. It will be modified
 * 			whenever new sensor data is received at any point in time.
 */
IMU* IMU__create(I2C_HandleTypeDef* i2cChannel, TIM_HandleTypeDef* timChannel, uint8_t timeBetweenSamples, volatile uint32_t* headingOutput);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------- IMU METHODS -------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * This function should be called when there is a I2C DMA transmit callback associated with this IMU sensor.
 * It will check that the I2C handle of the iterrupt matches that of the IMU and then respond accordingly.
 *
 * @param self The IMU object that transmitted
 * @param I2cHandle is the I2C handle of the interrupt
 */
void IMU__handleTxDMA(IMU* self, I2C_HandleTypeDef *I2cHandle);

/*
 * This function should be called when there is a I2C DMA receive callback associated with this IMU sensor.
 * It will check that the I2C handle of the iterrupt matches that of the IMU and then respond accordingly.
 * It will update the output registers of the iMU
 *
 * @param self The IMU object being read
 * @param I2cHandle is the I2C handle of the interrupt
 */
void IMU__handleRxDMA(IMU* self, I2C_HandleTypeDef *I2cHandle);


/*
 * This function should be called when there is an interrupt from the timer channel input in IMU_create.
 * It will check that the timer channel asscoiated with self is the same as timChannel and if it is
 * it will trigger the STM32 to get a measurement of the sensor data.
 *
 * @param self The IMU object that should be updated
 * @param timChannel The timer channel of the interrupt
 */
void IMU__updateBuffer(IMU* self, TIM_HandleTypeDef* timChannel);

#endif /* INC_IMU_H_ */
