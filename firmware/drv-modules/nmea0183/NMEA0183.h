/*
 * This library parses NMEA0183 messages when received by DMA. It performs sanity checks on the data and
 * provides methods for the user to extract the relevant data. More information on the NMEA0183 standard
 * can be found in IEC 61162-1:
 * 		1. https://www.chenyupeng.com/upload/2020/3/IEC%2061162-1-2010-728ad3778103426ab0a9a64b6cc5e474.pdf
 *
 * TODO: Add info about the high level function and where to put the IRQ handler.
 *
 * The data checks on incoming data do not fully comply to IEC 61162-1 Edition 4.0. Currently the following data checks
 * are implemented:
 * 		-Checksum
 * 		-Out of range characters
 * 		-Incorrect length of address field
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
#include <stdbool.h>

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define MAX_NMEA_CHANNELS 3 		//The maximum number of NMEA0183 channels. This is hardware limited based on there only being 3 UART channels. Don't change this value
#define BUFFER_SIZE 8 				//The number of bytes to buffer prior to calling an interrupt.
#define MAX_SENTENCE_LENGTH 127 	//The maximum number of bytes in a NMEA0183 sentence. The standard limit is 82, but some devices do not follow convention, hence extra room

#define MAX_DATA_BUFFER_SIZE 8

//Constants for different NMEA data types. This is encoded as follows:
//		00000000aaaaaaaabbbbbbbbcccccccc Where the bits are the ASCII abbreviation of the data type: "CBA".
#define MESSAGE_VDM 0x4D4456
#define MESSAGE_MWV 0x56574D
#define MESSAGE_XDR 0x524458

//enum for the result of the checks conducted on a message
typedef enum {
	GOOD_MESSAGE = 0,
	BAD_START_CHARACTER = 1,
	BAD_TERMINATION_SEQUENCE = 2,
	BAD_CHECK_SUM = 3
} MESSAGE_STATUS;


//The NMEA0183Raw data type
typedef struct {
	uint8_t scentenceData[MAX_SENTENCE_LENGTH];
	uint8_t scentenceLength;
} NMEA0183Raw;

//The NMEA0183 data type
typedef struct {
	UART_HandleTypeDef * huart;
	uint8_t receiveBuffers[2][MAX_SENTENCE_LENGTH + 1];
	uint8_t receiveBufferPosition;
	uint8_t dataReady;
	NMEA0183Raw dataBuffer[MAX_DATA_BUFFER_SIZE];
	volatile uint8_t dataBufferReadIndex;
	volatile uint8_t dataBufferWriteIndex;
	volatile bool overflowed;
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
NMEA0183* NMEA0183__create(UART_HandleTypeDef * huartChannel);

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
 * Return the number of items in the NMEA0183 buffer.
 *
 * @param self is an initialized NMEA0183 object
 * @return the number of items in the buffer.
 */
uint8_t NMEA0183__itemsInBuffer(NMEA0183* self);

/*
 * Checks if the NMEA0183 message is valid. (i.e. check sum, termination characters, etc.)
 *
 * @param raw is the NMEA0183 message to check
 * @return gives the status of the message
 */
MESSAGE_STATUS NMEA0183__checkMessage(NMEA0183Raw * raw);

/*
 * Gets the next item in the buffer. Does not remove it from the buffer!
 *
 * @param self is an initialized NMEA0183 object with at least one item in the buffer
 * @return a raw NMEA0183 object. Can only be muted until the read index is incremented, at which point it may get overwritten with new data.
 */
NMEA0183Raw * NMEA0183__getTopBufferItem(NMEA0183 * self);

/*
 * Increments the read index for the NMEA0183 message buffer.
 *
 * @param self is an initialized NMEA0183 object with at least one item in the buffer
 */
void NMEA0183__incrementReadIndex(NMEA0183 * self);

/*
 * Deals the interrupts for the NMEA0183 channel. Should be placed TODO: find best place this can be put
 *
 * @huart is the huart channel which triggered the interrupt.
 */
void NMEA0183__IRQHandler(UART_HandleTypeDef *huart);

/*
 * Retrieves a pointer to a specified field of the NMEA0183 message.
 *
 * @param self Must be an initialized NMEA0183 object
 * @param targetField Must be a target field that exists in this message. 0 returns the sentence data type.
 * @return A pointer the field data in an ASCII string terminated by the '\0' character.
 */
uint8_t* NMEA0183__getField(NMEA0183Raw* self, uint8_t targetField);

/*
 * Retrieves a unique hash for the message data type.
 *
 * @param self Must be an initialized NMEA0183 object
 * @return Is a unique value for each different message data type. Some hashes are defined above.
 */
uint32_t NMEA0183__getScentenceType(NMEA0183Raw* self);

#endif /* INC_NMEA0183_H_ */
