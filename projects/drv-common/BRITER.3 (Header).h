/*
 * This library provides an interface to communicate between the microcontroller and BRITER Encoders.
 * It currently has the following functionality:
 *      - A method to set up the encoder and define the data rate
 *      - Automatically update a position register
 *      - Zero the encoder
 *
 * For this library to work as intended, "BRITER__handleDMA()" must be called 
 * inside "HAL_UARTEx_RxEventCallback()".
 *
 *  Created on: Nov 14, 2024
 *      Author: Michael Greenough
 */

 #ifndef INC_BRITER_H_
 #define INC_BRITER_H_
 
 // The GPIO and pin controlling encoder power (user must define in ioc)
#define ENCODER_POWER_GPIO   GPIOG  // Change if needed
#define ENCODER_POWER_PIN    GPIO_PIN_1  // PG1 (D64, CN9 Pin 30)
 
 //----------------------------------------------------------------------------------------------------------------------------------------------------------------
 //--------------------------------------------------------------------------- INCLUDES ---------------------------------------------------------------------------
 //----------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 #include "stm32u5xx_hal.h"
 
 //------------------------------------------------------------------------------------------------------------------------------------------------------------------
 //--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
 //------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 /**
  * BRITER Encoder Object Structure
  */
 typedef struct {
     UART_HandleTypeDef * huart;
     volatile uint16_t * encoderRaw;  // Raw encoder data
     int16_t angleVal;                // 0-360 angle value
     int16_t passval;                 // Clamped value (-45 to 45)
     uint8_t * inputBuffer;
     uint32_t lastValidDataTime;       // Timestamp of last valid message
 } BRITER;
 
 // The address used for the encoder (0x01 is default)
 #define ENCODER_ADDRESS 0x01
 
 //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 //--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
 //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 /**
  * Creates a new BRITER object.
  *
  * @param huartChannel USART channel associated with this BRITER object. 
  * @param encoderRaw Pointer to store the latest encoder position (auto-updated).
  * @param samplePeriod Time between encoder updates in milliseconds (minimum 20ms).
  * @return Pointer to an initialized BRITER object.
  */
 BRITER* BRITER__create(UART_HandleTypeDef * huartChannel, uint16_t samplePeriod);

 
 //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 //----------------------------------------------------------------------------- BRITER METHODS ----------------------------------------------------------------------------
 //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 /**
  * Handles incoming UART data via DMA. This function should be called inside HAL_UARTEx_RxEventCallback().
  *
  * @param self Pointer to a BRITER object.
  * @param huart UART handle passed by the callback function.
  * @param size Number of bytes received via UART.
  */
 void BRITER__handleDMA(BRITER* self, UART_HandleTypeDef *huart, uint16_t size);
 
 /**
  * Computes the 0-360 degree angle based on encoder raw data.
  *
  * @param self Pointer to a BRITER object.
  */
 void BRITER__computeAngle(BRITER *self);
 

 /**
  * Computes the passval (clamped -45 to 45 angle).
  *
  * @param self Pointer to a BRITER object.
  */
 void BRITER__computePassval(BRITER *self);
 

 /**
  * Checks for errors such as encoder timeout or disconnection.
  *
  * @param self Pointer to a BRITER object.
  */
 void BRITER__checkErrors(BRITER* self);
 
 
 /**
  * Zeroes the encoder position. This function takes a non-negligible amount of time to execute.
  * 
  * ⚠️ **Warning:** This should only be used for tuning, **not during regular sailing.**
  *
  * @param self Pointer to a BRITER object.
  */
 void BRITER__zeroPosition(BRITER* self);


 /**
  * Retrieves raw encoder data.
  * 
  * @param self Pointer to a BRITER object.
  */
 uint16_t BRITER__getEncoderRaw(BRITER* self);

 void BRITER__powerCycle(BRITER* self);


 
 #endif /* INC_BRITER_H_ */
 