/*
 * BRITER.c
 *
 *  Created on: Nov 29, 2024
 *      Author: Michael Greenough
 */

#include "BRITER.h"

uint8_t * autoPositionCMD = {0x00, 0x06, 0x00, 0x01}; //Set the encoder to automatically return position data
uint8_t * encoderDataRate = {0x00, 0x07, 0x00, 0x32}; //Set the encoder return rate (currently 50ms, must no be less than 20ms)

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PFP ---------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Calculates the modbus CRC value.
 *
 * @param inputBuffer Is the string to calculate the CRC of.
 * @param length Is the length of the string to calculate the CRC of.
 * @return The CRC value
 */
uint32_t modbus_CRC(uint8_t * inputBuffer, uint16_t length);

void sendScentence(uint8_t * outputData, uint16_t outputLength);

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- HELPER FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

uint32_t modbus_CRC(uint8_t * inputBuffer, uint16_t length){
    //This entire function is ripped from the encoder manual, I don't know how it works or anything.
	uint32_t wcrc = 0xFFFF;
	for(uint16_t i = 0; i < length; i++){
		wcrc ^= (uint32_t)inputBuffer[i];
		for(uint8_t j = 0; j < 8; j++){
			if(wcrc & 0x0001){
				wcrc >>= 1;
				wcrc ^= 0xa001;
			} else {
				wcrc >>= 1;
			}
		}
	}
	return wcrc;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void BRITER__init(BRITER* self, IRQn_Type IRQn, USART_TypeDef * huartChannel, void (*dataHandler)(uint16_t), uint8_t DMAPriority, uint32_t baudRate) {
	self->dataHandler = (void *) dataHandler;

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

    // Enable UART idle line detection interrupt IDK IF I ACTUALLY NEED THIS LOL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    __HAL_UART_ENABLE_IT(&self->huart), UART_IT_IDLE);

    sendScentence(self, autoPositionCMD, 4);
    sendScentence(self, encoderDataRate, 4);// ADD RECEIVING PARSING AND ERROR HANDLING

    HAL_Delay(1000);

	HAL_UARTEx_ReceiveToIdle_DMA(&self->huart, self->inputBuffer, 16);
}

BRITER* BRITER__create(IRQn_Type IRQn, USART_TypeDef * huartChannel, void (*dataHandler)(uint16_t), uint8_t DMAPriority, uint32_t baudRate){
    BRITER* result = (BRITER*)malloc(sizeof(BRITER));
	NMEA0183__init(result, IRQn, huartChannel, dataHandler, DMAPriority, baudRate);
	return result;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- DATA FUNCTIONS -----------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void BRITER__handleDMA(BRITER* self, UART_HandleTypeDef *huart){//THIS WHOLE SECTION IS SHIT, NEEDS TO BE REPLACED
    uint8_t * bufPoint  = self->inputBuffer;
	uint16_t singleTurnValue;
	if(bufPoint[0] == 0x01 && bufPoint[1] == 0x03 && bufPoint[2] = 0x02){
		singleTurnValue = bufPoint[3];
		singleTurnValue << 8;
		singleTurnValue += bufPoint[4];

		(*(self->dataHandler))(singleTurnValue);
	}

	HAL_UARTEx_ReceiveToIdle_DMA(&self->huart, self->inputBuffer, 16);
}

uint8_t sendScentence(BRITER* self, uint8_t * outputData, uint16_t outputLength){
    uint8_t outputBuffer[16] = malloc(16);
    
    outputBuffer[0] = ENCODER_ADDRESS;
    outputBuffer[1] = 0x06;
    
    memcpy(outputBuffer + 2, outputData, outputLength);
    
    uint32_t checkSum =  modbus_CRC(outputBuffer, outputLength + 2);
    outputBuffer[outputLength + 2] = checkSum & 0xFF;
    outputBuffer[outputLength + 3] = (checkSum >> 8) & 0xFF;

    HAL_UART_Transmit(&(self->huart), outputBuffer, outputLength + 4, HAL_MAX_DELAY);

    free(outputBuffer);
}

