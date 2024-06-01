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
	self->goodData = 0;
	self->checkSum = 0;
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

	HAL_UART_Receive_DMA(&(self->huart), self->rxBuff, BUFFER_SIZE);
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


			//make sure we are not parsing from the same channel at the same time. Better to miss a sentence then unknown behavior due to time sharing
			if(self->dataReady == 0){
				//go through each byte in buffer
				for(int i = 0; i < BUFFER_SIZE; i++){

					if(self->rxBuff[i] == '$' || self->rxBuff[i] == '!'){ //check for sentence start, reset pointer and indicate good data
						self->sentencePosition = 1;
						self->goodData = 1;
						self->sentence[0] = self->rxBuff[i];
						self->checkSum = 0;

					} else if(self->goodData){
						uint8_t position = self->sentencePosition;
						uint8_t * sentencePointer = self->sentence;
						if((self->rxBuff[i] > 32 && self->rxBuff[i] < 126) || self->rxBuff[i] == 10 || self->rxBuff[i] == 13){//check that byte in range
							if(self->rxBuff[i] == 10){ // end of sentence
								if(position < MAX_SENTENCE_LENGTH){//if there is enough room add end of sentence
									sentencePointer[self->sentencePosition++] = 10;
									sentencePointer[self->sentencePosition++] = '\0';

									//remove the two extra characters from the check sum calculation "*\x0D" then check the check sum
									self->checkSum ^= 13 ^ 42 ^ sentencePointer[position-3] ^ sentencePointer[position-2];
									if(decimalToHexAscii(self->checkSum / 16) == sentencePointer[position-3] &&
											decimalToHexAscii(self->checkSum % 16) == sentencePointer[position-2]){

										//check proper ending sequence
										if(sentencePointer[position-1] == 13 &&
												sentencePointer[position-4] == '*'){


											if(strlen((const char *) &sentencePointer[1]) == 5){
												//Call data handler
												self->dataReady = 1;
												(*(self->dataHandler))(self);
												self->dataReady = 0;
											}
										}

									}
								}
								self->goodData = 0;
							} else {
								//if regular char update check sum and add it.
								self->checkSum ^= self->rxBuff[i];
								if(position < MAX_SENTENCE_LENGTH){
									if(self->rxBuff[i] != ',')
										sentencePointer[self->sentencePosition++] = self->rxBuff[i];
									else
										sentencePointer[self->sentencePosition++] = '\0';
								} else {
									self->goodData = 0;
								}
							}
						} else {
							self->goodData = 0;
						}
					}
				}
			}
			HAL_UART_Receive_DMA(huart, self->rxBuff, BUFFER_SIZE);
		}
	}
}

uint8_t* NMEA0183__getField(NMEA0183* self, uint8_t targetField) {
	if(self->dataReady){
		uint8_t i = 1;
		for (uint8_t field = 0; field < targetField; i++) {
			if (self->sentence[i] == '\0')
				field++;

			if (i >= MAX_SENTENCE_LENGTH)
				return NULL;
		}
		return &self->sentence[i];
	}
	return NULL;
}

uint32_t NMEA0183__getScentenceDataType(NMEA0183 * self) {
	if(self->dataReady){
		return (self->sentence[3] << 16) | (self->sentence[4] << 8) | self->sentence[5];
	}
	return UINT32_MAX;
}
