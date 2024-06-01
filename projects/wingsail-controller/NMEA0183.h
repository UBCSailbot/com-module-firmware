/*
 * This library parses NMEA0183 messages when received by DMA. It performs sanity checks on the data and
 * provides methods for the user to extract the relevant data. More information on the NMEA0183 standard
 * can be found in IEC 61162-1:
 * 		1. https://www.chenyupeng.com/upload/2020/3/IEC%2061162-1-2010-728ad3778103426ab0a9a64b6cc5e474.pdf
 *
 * For this library to work as intended "NMEA0183__handleDMA()" must be called when "HAL_UART_RxCpltCallback()" is called.
 *
 * The data checks on incoming data do not fully comply to IEC 61162-1 Edition 4.0. Currently the following data checks
 * are implemented:
 * 		-Checksum
 * 		-Out of range characters
 * 		-Incorrect length of address field
 * 		-Too long of sentence
 * 		-Improper termination character sequence
 *
 *  Created on: Apr 22, 2024
 *      Author: Michael Greenough
 */

#ifndef INC_NMEA0183_H_
#define INC_NMEA0183_H_

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- INCLUDES ---------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "stm32u5xx_hal.h"

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define MAX_NMEA_CHANNELS 3 	//The maximum number of NMEA0183 channels. This is hardware limited based on there only being 3 UART channels.
#define BUFFER_SIZE 8 			//The number of bytes to buffer prior to calling an interrupt.
#define MAX_SENTENCE_LENGTH 82 	//The maximum number of bytes in a NMEA0183 sentence. The standard limits it to 82.

//Constants for different NMEA data types. This is encoded as follows:
//		00000000aaaaaaaabbbbbbbbcccccccc Where the bits are the ASCII abbreviation of the data type: "ABC".
#define VDM 5653581
#define MWV 5068630
#define XDR 5784658

//The NMEA0183 data type
typedef struct {
	UART_HandleTypeDef huart;
	void (*dataHandler)(void * NMEA0183Pointer);
	uint8_t rxBuff[BUFFER_SIZE];
	uint8_t sentence[MAX_SENTENCE_LENGTH + 1];
	uint8_t sentencePosition;
	uint8_t checkSum;
	uint8_t goodData;
	uint8_t dataReady;
} NMEA0183;



//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Creates a new NMEA0183 object.
 *
 * @param IRQn Is an interrupt associated with the specified USART channel. The object must not be muted after calling this function.
 * @param huartChannel Is the USART channel associated with this AIS object. The object must not be muted after calling this function.
 * @param dataHandler Is called when a new sentence is received and the associated NMEA0183 object is passed as the parameter.
 * 							The associated function should be quick, otherwise sentences may be skipped when parsing
 * 							previous sentences. Only after this function has executed will the USART peripheral begin listening
 * 							for new data. Execution in <200us is recommended.
 * @param DMAPiority Is the priority of the DMA channel. The priority is DMAPriority / 16. The sub priority is DMAPriority % 16.
 * @param baudRate Is the baud rate of the connected device. For NMEA0183 this is either 4,800 or 38,400.
 * @return An initialized NMEA0183 object.
 */
NMEA0183* NMEA0183__create(IRQn_Type IRQn, USART_TypeDef * huartChannel, void (*dataHandler)(NMEA0183 *), uint8_t DMAPriority, uint32_t baudRate);

/*
 * Deletes the NMEA0183 object.
 *
 * @param self Must be an initialized NMEA0183 object
 */
void NMEA0183__destroy(NMEA0183* self);

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- NMEA0183 METHODS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * This function should be called when there is a UART DMA receive request associated with a NMEA channel.
 * This function may call the "dataHandler" function as passed in "NMEA0183__create"
 *
 * @param huart Is the UART data received from the DMA callback.
 */
void NMEA0183__handleDMA(UART_HandleTypeDef *huart);

/*
 * Retrieves a pointer to a specified field of the NMEA0183 message.
 * Can only be called from the function defined by the "dataHandler" function pointer in "NMEA0183__create()" or
 * a NULL pointer will be returned.
 *
 * @param self Must be an initialized NMEA0183 object
 * @param targetField Must be a target field that exists in this message. 0 returns the sentence data type.
 * @return A pointer the field data in an ASCII string terminated by the '\0' character.
 */
uint8_t* NMEA0183__getField(NMEA0183* self, uint8_t targetField);

/*
 * Retrieves a unique hash for the message data type.
 * Can only be called from the function defined by the "dataHandler" function pointer in "NMEA0183__create()" or
 * the return value will UINT32_MAX.
 *
 * @param self Must be an initialized NMEA0183 object
 * @return Is a unique value for each different message data type. Some hashes are defined above.
 */
uint32_t NMEA0183__getScentenceDataType(NMEA0183* self);

#endif /* INC_NMEA0183_H_ */
