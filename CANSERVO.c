
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "CANSERVO.h"
#include "CANSPI.h"

void set_servo_angle (float angle) {
    uint16_t servo_position = 0x2000 + (angle * 4096 / 90);
    send_can_msg(servo_position, REG_POSITION_NEW);

    return;
}

float check_servo_angle () {
    uint16_t servo_position = receive_can_msg(REG_POSITION_NEW);
    printf("Raw: %i",(int16_t) servo_position);
    float servo_angle = (servo_position - 0x2000) * (90.0/4096.0);

    return servo_angle;
}

uint8_t calculate_write_checksum (uint8_t ID, uint8_t address, uint8_t reg_length, uint8_t lo_byte, uint8_t hi_byte) {
  uint8_t write_checksum = (uint8_t) ((ID+address+reg_length+lo_byte+hi_byte) & 0xFF);

  return write_checksum;
}

uint8_t calculate_read_checksum (uint8_t ID, uint8_t address, uint8_t reg_length) {
  uint8_t read_checksum = (uint8_t) ((ID+address+reg_length) & 0xFF);

  return read_checksum;
}

void send_can_msg(uint16_t val, uint8_t reg) {

	uCAN_MSG txMessage;
	uint8_t lo_byte = val & 0xFF;
	uint8_t hi_byte = val >> 8;

	txMessage.frame.idType 	= dEXTENDED_CAN_MSG_ID_2_0B;
	txMessage.frame.id 		= SERVO_CAN_ID;
	txMessage.frame.dlc 	= SERVO_WRITE_DLC;
	txMessage.frame.data0 	= 0x96;
	txMessage.frame.data1 	= SERVO_ID;
	txMessage.frame.data2 	= reg;
	txMessage.frame.data3 	= SERVO_REG_LENGTH;
	txMessage.frame.data4 	= lo_byte;
	txMessage.frame.data5 	= hi_byte;
	txMessage.frame.data6 	= calculate_write_checksum (SERVO_ID, reg, SERVO_REG_LENGTH, lo_byte, hi_byte);

	CANSPI_Transmit(&txMessage);
	HAL_Delay(10);

}

uint16_t receive_can_msg(uint8_t reg) {

	uCAN_MSG txMessage;
	uCAN_MSG rxMessage;
	uint16_t reg_data = 0;

	txMessage.frame.idType 	= dEXTENDED_CAN_MSG_ID_2_0B;
	txMessage.frame.id 		= SERVO_CAN_ID;
	txMessage.frame.dlc 	= SERVO_READ_DLC;
	txMessage.frame.data0 	= 0x96;
	txMessage.frame.data1 	= SERVO_ID;
	txMessage.frame.data2 	= reg;
	txMessage.frame.data3 	= 0;
	txMessage.frame.data4 	= calculate_read_checksum (SERVO_ID, reg, SERVO_REG_LENGTH);


	CANSPI_Transmit(&txMessage);
	HAL_Delay(10);

	if(CANSPI_Receive(&rxMessage)) {

		if(rxMessage.frame.data0 == 0x69){
			printf("Received valid data");
			reg_data = (((uint16_t)rxMessage.frame.data5) << 8) | rxMessage.frame.data4;

			return reg_data;

		} else {

			return -1;
		}


	} else {

		return -2;
	}

}




