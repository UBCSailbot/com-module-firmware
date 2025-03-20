/*
 * WIND_SENSOR.c
 *
 * Created: March 20, 2025
 * Author: Faaiq Majeed
 * Organization: UBC Sailbot
 *
 */

#include "wind_sensor.h"
#include <stdio.h>

//Variables for processing wind sentence data
uint8_t rx_buffer[1] = {0};
uint8_t wind_sentence[WIND_BUFFER_SIZE]; //max sentence identifier length is 10 "PLCJEA870" + \0
int wind_sentence_index = 0;
uint8_t wind_speed[5]; //in knots
uint8_t wind_direction[5]; //in degrees
uint8_t wind_temperature[5]; //in Degrees Celsius

void processWindSensorData(uint8_t *input_data)
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

				//Wind Direction (Degrees)
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


				//Wind Speed (Knots)
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

				//UNCOMMENT THESE TO PRINT WIND DIRECTION, WIND SPEED, AND WIND TEMPERATURE TO PUTTY THROUGH USART1
//				printf("Wind Direction:%s\x0D\x0A", wind_direction);
//				printf("Wind Speed:%s\x0D\x0A", wind_speed);
//				printf("Wind Temperature:%s\x0D\x0A", wind_temperature);
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

