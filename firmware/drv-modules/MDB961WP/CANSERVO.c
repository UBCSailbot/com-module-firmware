
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "CANSERVO.h"
#include "CANSPI.h"

#if defined(CAN_SERVO_DEBUG)
#define CAN_SERVO_DEBUG_PRINT(...)                                             \
  do {                                                                         \
    if (g_fw_enable_debug_prints != 0U) {                                      \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (0)
#else
#define CAN_SERVO_DEBUG_PRINT(...) ((void)0)
#endif

void set_servo_angle (float angle) {
    uint16_t servo_position = 0x2000 + (angle * 4096 / 90);
    CAN_SERVO_DEBUG_PRINT("[SERVO] set angle=%.2f target_raw=%u reg=0x%02X\r\n",
                          angle, servo_position, REG_POSITION_NEW);
    send_can_msg(servo_position, REG_POSITION_NEW);

    return;
}

float check_servo_angle () {
    uint16_t servo_position = receive_can_msg(REG_POSITION_NEW);
    CAN_SERVO_DEBUG_PRINT("[SERVO] read raw=%d\r\n", (int16_t)servo_position);
    float servo_angle = (servo_position - 0x2000) * (90.0/4096.0);
    CAN_SERVO_DEBUG_PRINT("[SERVO] read angle=%.2f\r\n", servo_angle);

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

	CAN_SERVO_DEBUG_PRINT(
	    "[SERVO][TX] id=0x%03lX servo_id=%u reg=0x%02X val=%u lo=0x%02X hi=0x%02X "
	    "ck=0x%02X\r\n",
	    (unsigned long)txMessage.frame.id, txMessage.frame.data1, reg, val,
	    lo_byte, hi_byte, txMessage.frame.data6);

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

	CAN_SERVO_DEBUG_PRINT(
	    "[SERVO][TX-READ] id=0x%03lX servo_id=%u reg=0x%02X ck=0x%02X\r\n",
	    (unsigned long)txMessage.frame.id, txMessage.frame.data1, reg,
	    txMessage.frame.data4);

	CANSPI_Transmit(&txMessage);
	HAL_Delay(10);

	if(CANSPI_Receive(&rxMessage)) {
		CAN_SERVO_DEBUG_PRINT(
		    "[SERVO][RX] id=0x%03lX dlc=%u b0=0x%02X b1=0x%02X b2=0x%02X "
		    "b3=0x%02X b4=0x%02X b5=0x%02X\r\n",
		    (unsigned long)rxMessage.frame.id, rxMessage.frame.dlc,
		    rxMessage.frame.data0, rxMessage.frame.data1, rxMessage.frame.data2,
		    rxMessage.frame.data3, rxMessage.frame.data4, rxMessage.frame.data5);

		if(rxMessage.frame.data0 == 0x69){
			CAN_SERVO_DEBUG_PRINT("[SERVO][RX] valid response header\r\n");
			reg_data = (((uint16_t)rxMessage.frame.data5) << 8) | rxMessage.frame.data4;
			CAN_SERVO_DEBUG_PRINT("[SERVO][RX] reg_data=%u\r\n", reg_data);

			return reg_data;

		} else {
			CAN_SERVO_DEBUG_PRINT(
			    "[SERVO][RX] invalid response header=0x%02X returning -1\r\n",
			    rxMessage.frame.data0);

			return -1;
		}


	} else {
		CAN_SERVO_DEBUG_PRINT("[SERVO][RX] no response returning -2\r\n");

		return -2;
	}

}
