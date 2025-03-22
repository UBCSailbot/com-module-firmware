/*
 * WIND_SENSOR.c
 *
 * Created: March 20, 2025
 * Author: Faaiq Majeed
 * Organization: UBC Sailbot
 *
 */

#include "wind_sensor.h"

//Variables for processing wind sentence data
uint16_t rx_buffer[1] = {0};
uint16_t wind_sentence[WIND_BUFFER_SIZE]; //max sentence identifier length is 10 "PLCJEA870" + \0
int wind_sentence_index = 0;
uint16_t wind_speed[5]; //in knots
uint16_t wind_direction[4]; //in degrees
uint16_t wind_temperature[5]; //in Degrees Celsius
uint16_t wind_data_can_output[32]; //OUTPUT TO CANBUS

void processWindSensorData(uint16_t *input_data)
{
	if(input_data[0] == '$')
	{
		//First sentence, or finished sentence.
		if(wind_sentence_index != 0)
		{
			//completed wind sentence message, now process data
			if(wind_sentence[1] == 'I')
			{
				//Process <<Wind Sentence>>

				//Wind Direction (Degrees) - Resolution: 1 degree
				int k = 0;
				for(int i = 7; wind_sentence[i] != ','; i++)
				{
					wind_direction[k] = wind_sentence[i];
					k++;
					if(k >= 5)
					{
						//Wind Direction is always 5 indexes long
						return;
					}
				}
				for(int i = 0; i < 3; i++)
				{
					//We can ignore the decimal place since resolution = 1 degree
					wind_data_can_output[i] = ASCIItoINT(wind_direction[i]);
				}

				//Wind Speed (Knots) - Resolution 0.1 knots (Therefore, multiply by 10 in order to send as int and then SOFTWARE will divide by 10 on their side)
				k = 0;
				for(int i = 15; wind_sentence[i] != ','; i++)
				{
					wind_speed[k] = wind_sentence[i];
					k++;
					if(k >= 5)
					{
						//Wind Speed is always 5 indexes long
						return;
					}
				}
				for(int i = 0; i < 4; i++)
				{
					if(wind_speed[i] == '.')
					{
						//Skip the decimal point and the next bit is the decimal
						wind_data_can_output[i] = ASCIItoINT(wind_speed[i+1]);
					}
					else
					{
						wind_data_can_output[i] = ASCIItoINT(wind_speed[i]);
					}
				}
			}
			else if(wind_sentence[1] == 'W')
			{
				//Process <<Wind Temperature>>

				int k = 0;
				for(int i = 9; wind_sentence[i] != ','; i++)
				{
					wind_temperature[k] = wind_sentence[i];
					k++;
					if(k >= 5)
					{
						//Wind Temperature is always 5 indexes long
						return;
					}
				}
				//NO NEED TO ADD Wind Temperature TO CANBUS as of now.
				//If adding, it can be negative and the ASCIItoInt function must be changed.

				//UNCOMMENT THESE TO PRINT WIND DIRECTION, WIND SPEED, AND WIND TEMPERATURE TO PUTTY THROUGH USART1
//				printf("Wind Direction:%s\x0D\x0A", wind_direction);
//				printf("Wind Speed:%s\x0D\x0A", wind_speed);
//				printf("Wind Temperature:%s\x0D\x0A", wind_temperature);
			}
			else
			{
				//Either we received an invalid message or the 2 technical messages that are not required
				return;
			}
			wind_sentence_index = 0; //reset index for new message
			wind_sentence[wind_sentence_index] = input_data[0];
			wind_sentence_index++;
		}
		else
		{
			//For first sentence received
			wind_sentence[wind_sentence_index] = input_data[0];
			wind_sentence_index++;
		}
	}
	else
	{
		//gathering wind sentence data
		if(wind_sentence_index >= WIND_BUFFER_SIZE)
		{
			//Wind Sentence Should Not Be More Than Around 32 Characters Long (Usually it is around 26-29)
			return;
		}
		wind_sentence[wind_sentence_index] = input_data[0];
		wind_sentence_index++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
    	processWindSensorData(rx_buffer);
    }
    HAL_UART_Receive_DMA(&huart2, rx_buffer, 1);
}

int ASCIItoINT(uint16_t *input)
{
	//Assumes all inputs must be positive
	//Since all wind direction, speed and temperature are 5 bytes long (3 numbers, then a decimal and then a number, ex: 125.2). We know it will be
	if(input[0] >= '0' && input[0] <= '9')
	{
		return (input[0] - '0');
	}
}

