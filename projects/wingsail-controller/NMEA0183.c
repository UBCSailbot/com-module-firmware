/*
 * NMEA0183.c
 *
 *  Created on: Apr 22, 2024
 *      Author: Michael Greenough
 */

#include "NMEA0183.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

//This array provides a way to find the associated NEMA0183 object with a UART interface.
void * huartNMEALookup[MAX_NMEA_CHANNELS][2] = {{NULL, NULL},{NULL, NULL},{NULL, NULL}};

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PFP ---------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Converts from a decimal number to a HEX number with ASCII encoding.
 *
 * @param decmial Is a decimal number in the range [0,15]
 * @return The ASCII code of the associated hex character. Takes the range of ['0','F']
 */
uint8_t decimalToHexAscii(uint8_t decimal);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
NMEA0183* NMEA0183__create(UART_HandleTypeDef * huartChannel){
	NMEA0183* result = (NMEA0183*)malloc(sizeof(NMEA0183));
	memset(result, 0, sizeof(NMEA0183));

	for(int i = 0; i < MAX_NMEA_CHANNELS; i++){
		if(huartNMEALookup[i][0] == NULL){
			huartNMEALookup[i][0] = huartChannel->Instance;
			huartNMEALookup[i][1] = result;
			break;
		} else if(huartNMEALookup[i][0] == huartChannel->Instance){
			huartNMEALookup[i][1] = result;
			break;
		}
	}

	result->huart = huartChannel;

	CLEAR_BIT(huartChannel->Instance->CR1, USART_CR1_UE);  		// Disable USART
	MODIFY_REG(huartChannel->Instance->CR2, USART_CR2_ADD,
				('\x0A' << UART_CR2_ADDRESS_LSB_POS));	// Modify the CR2 register to have the character to match in the 8 MSB
	__HAL_UART_ENABLE_IT(huartChannel, UART_IT_CM);				// Enable the character match interrupt
	SET_BIT(huartChannel->Instance->CR1, USART_CR1_UE);    		// Re-enable USART

	HAL_UART_Receive_DMA(huartChannel, result->receiveBuffers[result->receiveBufferPosition], MAX_SENTENCE_LENGTH + 1);

	return result;
}

void NMEA0183__destroy(NMEA0183* self){
	if (self) {
		free(self);
	}
}

void NMEA0183__IRQHandler(UART_HandleTypeDef *huart){
	for(int i = 0; i < MAX_NMEA_CHANNELS; i++){
		if(huartNMEALookup[i][0] == huart->Instance){
			if(READ_BIT(huart->Instance->ISR, USART_ISR_CMF)){ //Check if character matches
				NMEA0183 * nmea = huartNMEALookup[i][1];

				//stop receiving data
				HAL_UART_DMAStop(huart);

				//record message length
				uint16_t receivedLength = MAX_SENTENCE_LENGTH + 1 - __HAL_DMA_GET_COUNTER(huart->hdmarx);

				//Clear the character match interrupt flag
				WRITE_REG(huart->Instance->ICR, USART_ICR_CMCF);

				//Swap to the other buffer for reading while we copy the current message
				nmea->receiveBufferPosition = 1 - nmea->receiveBufferPosition;

				//Restart the reception
				HAL_UART_Receive_DMA(huart,
						nmea->receiveBuffers[nmea->receiveBufferPosition],
						MAX_SENTENCE_LENGTH + 1);

				//check that message is less than the max allowable size
				if(receivedLength > MAX_SENTENCE_LENGTH){ //ADD MIN LENGTH
					Error_Handler();
				}

				//get the index of the next write position
				uint8_t next = (nmea->dataBufferWriteIndex + 1) % MAX_DATA_BUFFER_SIZE;

				//check that we have room in the buffer
				if (next != nmea->dataBufferReadIndex) {

					//copy the raw message to the data buffer
					memcpy(nmea->dataBuffer[nmea->dataBufferWriteIndex].scentenceData,
							nmea->receiveBuffers[1 - nmea->receiveBufferPosition],
							receivedLength);

					//copy the message length to the data buffer
				    nmea->dataBuffer[nmea->dataBufferWriteIndex].scentenceLength = receivedLength;

				    //Set the next write index
				    nmea->dataBufferWriteIndex = next;
				} else {
					//Buffer full dropping message
					Error_Handler();
				}

			} else {
				//No CM IRQ (should not happen)
				Error_Handler();
			}
			break;
		}
	}
}

uint8_t NMEA0183__itemsInBuffer(NMEA0183* self) {
    return (self->dataBufferWriteIndex - self->dataBufferReadIndex) % MAX_DATA_BUFFER_SIZE;
}

NMEA0183Raw * NMEA0183__getTopBufferItem(NMEA0183 * self) {
	if(NMEA0183__itemsInBuffer(self) > 0){
		return &self->dataBuffer[self->dataBufferReadIndex];
	}
	return NULL;
}

void NMEA0183__incrementReadIndex(NMEA0183 * self) {
	if(NMEA0183__itemsInBuffer(self) > 0){
		self->dataBufferReadIndex = (self->dataBufferReadIndex + 1) % MAX_DATA_BUFFER_SIZE;
	}
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- HELPER FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

uint8_t decimalToHexAscii(uint8_t decimal){
	if(decimal < 10){
		return 48 + decimal;
	}
	return 55 + decimal;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- DATA PARSING FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

uint8_t NMEA0183__checkMessage(NMEA0183Raw * inputMessage){
	uint8_t startCharacter = inputMessage->scentenceData[0];
	//Check start characters
	if(startCharacter != '!' && startCharacter != '$')
		return BAD_START_CHARACTER;

	//Check end characters
	if(inputMessage->scentenceData[inputMessage->scentenceLength - 1] != '\x0A'
			|| inputMessage->scentenceData[inputMessage->scentenceLength - 2] != '\x0D'
					|| inputMessage->scentenceData[inputMessage->scentenceLength - 5] != '*')
		return BAD_TERMINATION_SEQUENCE;

	//verify check sum
	uint8_t checkSum = 0;
	for(int i = 1; i < inputMessage->scentenceLength - 5; i++){
		checkSum ^= inputMessage->scentenceData[i];
		if(inputMessage->scentenceData[i] == ',')
			inputMessage->scentenceData[i] = '\0';
	}

	if(decimalToHexAscii(checkSum / 16) != inputMessage->scentenceData[inputMessage->scentenceLength - 4]
								|| decimalToHexAscii(checkSum % 16) != inputMessage->scentenceData[inputMessage->scentenceLength - 3])
		return BAD_CHECK_SUM;

	//Change the * to a \0 for the field return format
	inputMessage->scentenceData[inputMessage->scentenceLength - 5] = '\0';

	return GOOD_MESSAGE;
}

uint8_t* NMEA0183__getField(NMEA0183Raw* data, uint8_t targetField) {
	uint8_t messageIndex = 1;
	for (uint8_t field = 0; field < targetField; messageIndex++) {
		if (data->scentenceData[messageIndex] == '\0')
			field++;
		if (messageIndex >= MAX_SENTENCE_LENGTH || messageIndex == 255 || data->scentenceData[messageIndex] == '\x0D')
			return NULL;
	}
	return &data->scentenceData[messageIndex];
}


uint32_t NMEA0183__getScentenceType(NMEA0183Raw * data) {
	return ((uint32_t *) &data->scentenceData[3])[0] & 0x00FFFFFF;
	//return (self->receiveBuffer[self->scentenceStartPosition + 3] << 16) | (self->receiveBuffer[self->scentenceStartPosition + 4] << 8) | self->receiveBuffer[self->scentenceStartPosition + 5];
}


//void NMEA0183__handleDMA(UART_HandleTypeDef *huart)
//{
//	//Find which NMEA0183 object corresponds to the uart channel
//	for(int i = 0; i < MAX_NMEA_CHANNELS; i++){
//		if(huartNMEALookup[i][0] == huart->Instance){
//
//
//			NMEA0183* self = huartNMEALookup[i][1];
//			int16_t startCharPosition = -1;
//			if(self->receivePosition > 20){//163
//				for(int i = self->receivePosition; i < self->receivePosition + 8; i++){
//					if(self->receiveBuffer[i] == '!' || self->receiveBuffer[i] == '$'){
//						startCharPosition = i;
//						self->receivePosition += 8 - startCharPosition;
//						break;
//					}
//				}
//			}
//
//			if(startCharPosition == -1){
//				self->receivePosition += 8;
//				self->receivePosition %= 256;
//			}
//
//			HAL_UART_Receive_DMA(&self->huart, self->receiveBuffer + self->receivePosition, 8);
//
//			if(startCharPosition != -1){
//				for(int i = 0; i < self->receivePosition; i++){
//					self->receiveBuffer[i] = self->receiveBuffer[startCharPosition + i];
//				}
//
//				for(int i = self->receivePosition - 8 + startCharPosition; i < startCharPosition; i++){
//					checkSpecialChars(self,i);
//				}
//
//				for(int i = 0; i < self->receivePosition; i++){
//					checkSpecialChars(self,i);
//				}
//			} else {
//				for(int i = self->receivePosition - 8; i < self->receivePosition; i++){
//					checkSpecialChars(self,i);
//				}
//			}
//		}
//	}
//}

//void checkData(NMEA0183* self, uint8_t length){
//
//	uint8_t startCharacter = self->receiveBuffer[self->scentenceStartPosition];
//	if(startCharacter != '!' && startCharacter != '$')
//		return;
//
//	if(self->receiveBuffer[self->scentenceStartPosition + length] != '\x0A'
//			|| self->receiveBuffer[self->scentenceStartPosition + length - 1] != '\x0D'
//					|| self->receiveBuffer[self->scentenceStartPosition + length - 4] != '*')
//		return;
//
//	uint8_t checkSum = 0;
//	for(int i = self->scentenceStartPosition + 1; i < self->scentenceStartPosition + length - 4; i++){
//		checkSum ^= self->receiveBuffer[i];
//		if(self->receiveBuffer[i] == ',')
//			self->receiveBuffer[i] = '\0';
//	}
//	if(decimalToHexAscii(checkSum / 16) != self->receiveBuffer[self->scentenceStartPosition + length - 3]
//																|| decimalToHexAscii(checkSum % 16) != self->receiveBuffer[self->scentenceStartPosition + length - 2])
//		return;
//
//	self->dataReady = 1;
//	(*(self->dataHandler))(self);
//	self->dataReady = 0;
//}
//
//void checkSpecialChars(NMEA0183* self, uint8_t index){
//	if(self->receiveBuffer[index] == '\x0A'){
//		checkData(self, index - self->scentenceStartPosition);
//	} else if(self->receiveBuffer[index] == '!' || self->receiveBuffer[index] == '$'){
//		self->scentenceStartPosition = index;
//	}
//}
//
//uint8_t* NMEA0183__getField(NMEA0183* self, uint8_t targetField) {
//	if(self->dataReady){
//		uint8_t i = self->scentenceStartPosition;
//		for (uint8_t field = 0; field < targetField; i++) {
//			if (self->receiveBuffer[i] == '\0')
//				field++;
//
//			if (i >= MAX_SENTENCE_LENGTH || i == 255)
//				return NULL;
//		}
//		return &self->receiveBuffer[i];
//	}
//	return NULL;
//}
//
//uint32_t NMEA0183__getScentenceDataType(NMEA0183 * self) {
//	if(self->dataReady){
//		return (self->receiveBuffer[self->scentenceStartPosition + 3] << 16) | (self->receiveBuffer[self->scentenceStartPosition + 4] << 8) | self->receiveBuffer[self->scentenceStartPosition + 5];
//	}
//	return UINT32_MAX;
//}
