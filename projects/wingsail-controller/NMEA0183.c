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

void checkSpecialChars(NMEA0183* self, uint8_t inputChar);

void checkData(NMEA0183* self, uint8_t length);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void NMEA0183__init(NMEA0183* self, IRQn_Type IRQn, USART_TypeDef * huartChannel, void (*dataHandler)(NMEA0183), uint8_t DMAPriority, uint32_t baudRate) {
	for(int i = 0; i < MAX_NMEA_CHANNELS; i++){
		if(huartNMEALookup[i][0] == NULL){
			huartNMEALookup[i][0] = huartChannel;
			huartNMEALookup[i][1] = self;
			break;
		} else if(huartNMEALookup[i][0] == huartChannel){
			huartNMEALookup[i][1] = self;
			break;
		}
	}

	self->dataHandler = (void *) dataHandler;
	self->dataReady = 0;

	__HAL_RCC_GPDMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(IRQn);

	self->huart.Instance = huartChannel;
	self->huart.Init.BaudRate = baudRate;
	self->huart.Init.WordLength = UART_WORDLENGTH_8B;
	self->huart.Init.StopBits = UART_STOPBITS_1;
	self->huart.Init.Parity = UART_PARITY_NONE;
	self->huart.Init.Mode = UART_MODE_TX_RX;
	self->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	self->huart.Init.OverSampling = UART_OVERSAMPLING_16;
	self->huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	self->huart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	self->huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&self->huart) != HAL_OK)
	{
		//error handling
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&self->huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		//error handling
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&self->huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		//error handling
	}
	if (HAL_UARTEx_DisableFifoMode(&self->huart) != HAL_OK)
	{
		//error handling
	}

	HAL_UART_Receive_DMA(&(self->huart), self->receiveBuffer, 8);
}

NMEA0183* NMEA0183__create(IRQn_Type IRQn, USART_TypeDef * huartChannel, void (*dataHandler)(NMEA0183 *), uint8_t DMAPriority, uint32_t baudRate){
	NMEA0183* result = (NMEA0183*)malloc(sizeof(NMEA0183));
	memset(result, 0, sizeof(NMEA0183));
	NMEA0183__init(result, IRQn, huartChannel, dataHandler, DMAPriority, baudRate);
	return result;
}

void NMEA0183__reset(NMEA0183* self) {
}

void NMEA0183__destroy(NMEA0183* self){
	if (self) {
		NMEA0183__reset(self);
		free(self);
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

void NMEA0183__handleDMA(UART_HandleTypeDef *huart)
{
	//Find which NMEA0183 object corresponds to the uart channel
	for(int i = 0; i < MAX_NMEA_CHANNELS; i++){
		if(huartNMEALookup[i][0] == huart->Instance){


			NMEA0183* self = huartNMEALookup[i][1];
			int16_t startCharPosition = -1;
			if(self->receivePosition > 20){//163
				for(int i = self->receivePosition; i < self->receivePosition + 8; i++){
					if(self->receiveBuffer[i] == '!' || self->receiveBuffer[i] == '$'){
						startCharPosition = i;
						self->receivePosition += 8 - startCharPosition;
						break;
					}
				}
			}

			if(startCharPosition == -1){
				self->receivePosition += 8;
				self->receivePosition %= 256;
			}

			HAL_UART_Receive_DMA(&self->huart, self->receiveBuffer + self->receivePosition, 8);

			if(startCharPosition != -1){
				for(int i = 0; i < self->receivePosition; i++){
					self->receiveBuffer[i] = self->receiveBuffer[startCharPosition + i];
				}

				for(int i = self->receivePosition - 8 + startCharPosition; i < startCharPosition; i++){
					checkSpecialChars(self,i);
				}

				for(int i = 0; i < self->receivePosition; i++){
					checkSpecialChars(self,i);
				}
			} else {
				for(int i = self->receivePosition - 8; i < self->receivePosition; i++){
					checkSpecialChars(self,i);
				}
			}
		}
	}
}

void checkData(NMEA0183* self, uint8_t length){

	uint8_t startCharacter = self->receiveBuffer[self->scentenceStartPosition];
	if(startCharacter != '!' && startCharacter != '$')
		return;

	if(self->receiveBuffer[self->scentenceStartPosition + length] != '\x0A'
			|| self->receiveBuffer[self->scentenceStartPosition + length - 1] != '\x0D'
					|| self->receiveBuffer[self->scentenceStartPosition + length - 4] != '*')
		return;

	uint8_t checkSum = 0;
	for(int i = self->scentenceStartPosition + 1; i < self->scentenceStartPosition + length - 4; i++){
		checkSum ^= self->receiveBuffer[i];
		if(self->receiveBuffer[i] == ',')
			self->receiveBuffer[i] = '\0';
	}
	if(decimalToHexAscii(checkSum / 16) != self->receiveBuffer[self->scentenceStartPosition + length - 3]
																|| decimalToHexAscii(checkSum % 16) != self->receiveBuffer[self->scentenceStartPosition + length - 2])
		return;

	self->dataReady = 1;
	(*(self->dataHandler))(self);
	self->dataReady = 0;
}

void checkSpecialChars(NMEA0183* self, uint8_t index){
	if(self->receiveBuffer[index] == '\x0A'){
		checkData(self, index - self->scentenceStartPosition);
	} else if(self->receiveBuffer[index] == '!' || self->receiveBuffer[index] == '$'){
		self->scentenceStartPosition = index;
	}
}

uint8_t* NMEA0183__getField(NMEA0183* self, uint8_t targetField) {
	if(self->dataReady){
		uint8_t i = self->scentenceStartPosition;
		for (uint8_t field = 0; field < targetField; i++) {
			if (self->receiveBuffer[i] == '\0')
				field++;

			if (i >= MAX_SENTENCE_LENGTH || i == 255)
				return NULL;
		}
		return &self->receiveBuffer[i];
	}
	return NULL;
}

uint32_t NMEA0183__getScentenceDataType(NMEA0183 * self) {
	if(self->dataReady){
		return (self->receiveBuffer[self->scentenceStartPosition + 3] << 16) | (self->receiveBuffer[self->scentenceStartPosition + 4] << 8) | self->receiveBuffer[self->scentenceStartPosition + 5];
	}
	return UINT32_MAX;
}
