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
 *      Author: Michael Greenough, Christian Conroy
 */

 #ifndef INC_BRITER_H_
 #define INC_BRITER_H_
 
 // The GPIO and pin controlling encoder power (user must define in ioc)
 #define ENCODER_POWER_GPIO        GPIOG
 #define ENCODER_POWER_PIN         GPIO_PIN_0
 
 #define LED_BLUE_Pin              GPIO_PIN_7
 #define LED_BLUE_GPIO_Port       GPIOB
 
 //--------------------------------------------------------------------------------------------------------------------
 // Includes
 //--------------------------------------------------------------------------------------------------------------------
 
 #include "stm32u5xx_hal.h"
 #include <stdbool.h>
 
 //--------------------------------------------------------------------------------------------------------------------
 // Structures
 //--------------------------------------------------------------------------------------------------------------------
 
 /**
  * BRITER Encoder Object Structure
  */
 typedef struct {
     UART_HandleTypeDef *huart;       // UART handle for communication
     volatile uint16_t *encoderRaw;   // Pointer to raw encoder value (auto-updated)
     int16_t angleVal;                // Last computed angle (0-360 degrees)
     int16_t passval;                 // Clamped angle value (-45 to 45 degrees)
     uint8_t *inputBuffer;            // DMA input buffer
     uint32_t lastValidDataTime;      // Timestamp of last valid reception
     uint16_t lastRawValue;           // Last known raw value for error detection
     uint16_t samplePeriod;           // Encoder polling interval in milliseconds
     bool encoderReady;               // Encoder ready state
     uint32_t startupTime;            // Timestamp of initialization
 } BRITER;
 
 // Encoder address (default is 0x01)
 #define ENCODER_ADDRESS 0x01
 
 //--------------------------------------------------------------------------------------------------------------------
 // Object Management
 //--------------------------------------------------------------------------------------------------------------------
 
 /**
  * Creates and initializes a new BRITER encoder object.
  *
  * @param huartChannel   UART channel used to communicate with encoder.
  * @param samplePeriod   Polling period in milliseconds (minimum 20ms).
  * @return Pointer to the initialized BRITER object.
  */
 BRITER* BRITER__create(UART_HandleTypeDef *huartChannel, uint16_t samplePeriod);
 
 //--------------------------------------------------------------------------------------------------------------------
 // BRITER Methods
 //--------------------------------------------------------------------------------------------------------------------
 
 /**
  * Handles incoming UART data via DMA.
  * Should be called inside HAL_UARTEx_RxEventCallback().
  *
  * @param self    Pointer to BRITER object.
  * @param huart   UART handle provided by callback.
  * @param size    Number of bytes received.
  */
 void BRITER__handleDMA(BRITER *self, UART_HandleTypeDef *huart, uint16_t size);
 
 /**
  * Computes angle (0-360 degrees) from raw encoder value.
  *
  * @param self Pointer to BRITER object.
  * @return Computed angle.
  */
 int16_t BRITER__computeAngle(BRITER *self);
 
 /**
  * Computes clamped angle (-45 to 45 degrees).
  *
  * @param self Pointer to BRITER object.
  * @return Clamped angle.
  */
 int16_t BRITER__clampAngle(BRITER *self);
 
 /**
  * Checks for timeout and encoder communication errors.
  *
  * @param self Pointer to BRITER object.
  */
 void BRITER__checkErrors(BRITER *self);
 
 /**
  * Sends a zeroing command to the encoder.
  * ⚠️ Warning: Only use this during setup or calibration.
  *
  * @param self Pointer to BRITER object.
  */
 void BRITER__zeroPosition(BRITER *self);
 
 /**
  * Returns latest raw encoder reading.
  *
  * @param self Pointer to BRITER object.
  * @return Raw encoder value.
  */
 uint16_t BRITER__getEncoderRaw(BRITER *self);
 
 /**
  * Cuts and restores encoder power to recover from error state.
  *
  * @param self Pointer to BRITER object.
  */
 void BRITER__powerCycle(BRITER *self);
 
 /**
  * Blinks onboard LED (used as activity indicator).
  */
 void BRITER__blinkStatusLED(void);
 
 /**
  * Restarts DMA reception manually.
  *
  * @param self Pointer to BRITER object.
  */
 void BRITER__restartDMA(BRITER *self);
 
 /**
  * Reinitializes encoder communication and state.
  *
  * @param self Pointer to BRITER object.
  */
 void BRITER__reEngage(BRITER *self);
 
 #endif /* INC_BRITER_H_ */
 
