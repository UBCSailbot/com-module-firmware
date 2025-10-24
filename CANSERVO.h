
#ifndef CANSERVO_H
#define CANSERVO_H

// Register definitions
#define REG_POSITION_NEW    	0x1E
#define REG_POSITION_MAX_LIMIT	0xB0
#define REG_POSITION_MIN_LIMIT	0xB2

// Servo setup definitions
#define SERVO_CAN_ID		0x000
#define SERVO_ID			0
#define SERVO_WRITE_DLC		7
#define SERVO_READ_DLC		5
#define SERVO_REG_LENGTH	2

// Servo config definitions
#define SERVO_MAX_LIMIT     16383
#define SERVO_MIN_LIMIT     0


void set_servo_angle(float angle);
float check_servo_angle();
uint8_t calculate_write_checksum(uint8_t ID, uint8_t address, uint8_t reg_length, uint8_t lo_byte, uint8_t hi_byte);
uint8_t calculate_read_checksum(uint8_t ID, uint8_t address, uint8_t reg_length);
void send_can_msg(uint16_t val, uint8_t reg);
uint16_t receive_can_msg(uint8_t reg);

#endif
