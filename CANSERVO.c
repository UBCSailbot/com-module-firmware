#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "CANSERVO.h"
#include "CANSPI.h"

uint16_t servo_midpoint = 8192;


/**
 * @file CANSERVO.c
 * @brief CAN-servo driver: CAN message helpers + high-level servo commands.
 *
 * General notes:
 * - High-level setters/readers frequently call SERVO_OK() and reset the MCU if false.
 *   That means ANY detected servo fault (or CAN comms failure interpreted as fault)
 *   can immediately reboot the STM32. If you later move policy to main.c, you can
 *   keep this file purely as a driver layer.
 *
 * - Many functions accept float but ultimately send uint16 values over CAN.
 *   This means values are truncated when converted to integer (implementation-defined
 *   rounding in C casts is truncation toward zero). Keep this in mind when documenting
 *   min/max and units.
 */

// ============================================================================
// CAN Handling
// ============================================================================

/**
 * @brief Helper function to calculate write checksums when sending CAN messages.
 *
 * Checksum scheme used by your protocol implementation:
 *   checksum = (ID + address + reg_length + lo_byte + hi_byte) & 0xFF
 *
 * @param ID         Servo node ID (SERVO_ID).
 * @param address    Servo register address.
 * @param reg_length Register length in bytes (SERVO_REG_LENGTH, typically 2).
 * @param lo_byte    Low byte of payload.
 * @param hi_byte    High byte of payload.
 * @return 8-bit checksum.
 */
uint8_t calculate_write_checksum (uint8_t ID, uint8_t address, uint8_t reg_length, uint8_t lo_byte, uint8_t hi_byte) {
  uint8_t write_checksum = (uint8_t) ((ID+address+reg_length+lo_byte+hi_byte) & 0xFF);

  return write_checksum;
}

/**
 * @brief Helper function to calculate read checksums when receiving CAN messages.
 *
 * Checksum scheme used by your protocol implementation:
 *   checksum = (ID + address + reg_length) & 0xFF
 *
 * @param ID         Servo node ID (SERVO_ID).
 * @param address    Servo register address.
 * @param reg_length Register length in bytes (SERVO_REG_LENGTH, typically 2).
 * @return 8-bit checksum.
 */
uint8_t calculate_read_checksum (uint8_t ID, uint8_t address, uint8_t reg_length) {
  uint8_t read_checksum = (uint8_t) ((ID+address+reg_length) & 0xFF);

  return read_checksum;
}

/**
 * @brief Handles sending CAN messages (16-bit register write).
 *
 * Message format (as implemented):
 *  - Extended CAN ID (frame.id): SERVO_CAN_ID
 *  - DLC: SERVO_WRITE_DLC
 *  - data0: 0x96 (command header)
 *  - data1: SERVO_ID (servo device ID)
 *  - data2: reg (register address)
 *  - data3: SERVO_REG_LENGTH (register length in bytes)
 *  - data4: payload LSB
 *  - data5: payload MSB
 *  - data6: write checksum
 *
 * @param val 16-bit raw register value.
 * @param reg Register address to write.
 *
 * @note Uses CANSPI_Transmit() and prints "Values sent..." on success.
 * @note HAL_Delay(10) is used after transmit (protocol settle time).
 */
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

	uint8_t can_response = CANSPI_Transmit(&txMessage);

	if (can_response == 0){
		return;
	}

	HAL_Delay(10);
	return;

}

/**
 * @brief Handles receiving CAN messages (16-bit register read).
 *
 * Request format (as implemented):
 *  - Extended CAN ID (frame.id): SERVO_CAN_ID
 *  - DLC: SERVO_READ_DLC
 *  - data0: 0x96
 *  - data1: SERVO_ID
 *  - data2: reg
 *  - data3: 0
 *  - data4: read checksum (ID + reg + reg_length) & 0xFF
 *
 * Response handling (as implemented):
 *  - CANSPI_Receive() must succeed
 *  - rxMessage.frame.data0 must equal 0x69 (expected response header)
 *  - returned value is little-endian: data4 = LSB, data5 = MSB
 *
 * @param reg Register address to read.
 * @return Raw 16-bit register value.
 *
 * @warning On failure paths this function returns -1 or -2, but the return type is uint16_t.
 *          That means callers will receive 0xFFFF or 0xFFFE, which can look like "all bits set".
 *          This is especially risky for status registers like REG_EMERGENCY_STOP.
 */
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
			reg_data = (((uint16_t)rxMessage.frame.data5) << 8) | rxMessage.frame.data4;

			return reg_data;

		} else {

			// NOTE: returns 0xFFFF due to uint16_t return type.
			return -1;
		}


	} else {

		// NOTE: returns 0xFFFE due to uint16_t return type.
		return -2;
	}

}



// ============================================================================
// Servo Initialization
// ============================================================================

/**
 * @brief Sets initial values for the CAN SERVO upon start-up.
 *
 * Writes (via set_* wrappers):
 *  - SERVO_MAX_POSITION_LIMIT -> REG_POSITION_MAX_LIMIT (soft max limit)
 *  - SERVO_MIN_POSITION_LIMIT -> REG_POSITION_MIN_LIMIT (soft min limit)
 *  - SERVO_MAX_CURRENT        -> REG_CURRENT_MAX (max current limit)
 *
 * @note Ensure SERVO_* macros match safe limits from the servo datasheet.
 */
void servo_init(void) {

	set_servo_max_position_limit(SERVO_MAX_POSITION_LIMIT);
	set_servo_min_position_limit(SERVO_MIN_POSITION_LIMIT);
	set_servo_max_voltage(SERVO_MAX_VOLTAGE);
	set_servo_min_voltage(SERVO_MIN_VOLTAGE);
	set_servo_max_current(SERVO_MAX_CURRENT);

	servo_midpoint = 8192;


	return;
}



// ============================================================================
// Servo Positioning
// ============================================================================

/**
 * @brief Sets the position of the servo's arm (angle command).
 *
 * Register written: REG_POSITION_NEW
 *
 * Encoding used in this code:
 *  - servo_position = 0x2000 + angle * 4096 / 90
 * This implies:
 *  - 4096 counts = 90 degrees
 *  - 0x2000 corresponds to "0 degrees" reference
 *
 * @param angle Angle in degrees.
 * @note Comment says valid input is [-180, +180] degrees, but there is no clamp here.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_angle (float angle) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

    uint16_t servo_position = servo_midpoint + (angle * 4096 / 90);
    send_can_msg(servo_position, REG_POSITION_NEW);

    return;
}


/**
 * @brief Returns the servo arm's angle position (angle readback).
 *
 * Register read: REG_POSITION_NEW (same register used for writing command here)
 * Conversion used:
 *  - angle_deg = (raw - 0x2000) * (90 / 4096)
 *
 * @return Angle in degrees.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
float check_servo_angle (void) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return -1;
	}

    uint16_t servo_position = receive_can_msg(REG_POSITION_NEW);
    float servo_angle = (servo_position - servo_midpoint) * (90.0/4096.0);

    return servo_angle;
}

/**
 * @brief Set CAN servo mid position.
 *
 * Register written: REG_POSITION_MID
 *
 * Data range/encoding note (from your comment):
 *  - Data = 0..16383
 *  - Resolution: 4096 = 90 degrees
 *
 * @param servo_midpoint Midpoint value (raw units per servo firmware).
 * @warning Marked TODO / "does not actually work as intended" in your notes.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */

void set_servo_midpoint (void)
{
    if (!SERVO_OK()) {
        servo_error_handler();
        return;
    }

    // Read current raw position (counts)
    uint16_t raw_pos = receive_can_msg(REG_POSITION_NEW);

    // Set new "zero" reference
    servo_midpoint = raw_pos;

    // Shift endpoints with wrap-around (16-bit overflow allowed as requested)
    uint16_t new_max = (uint16_t)(servo_midpoint + 4096u);
    uint16_t new_min = (uint16_t)(servo_midpoint - 4096u);

    // Apply new soft limits
    set_servo_max_position_limit((float)new_max);
    set_servo_min_position_limit((float)new_min);

    return;
}


/**
 * @brief Returns the servo's mid point.
 *
 * Register read: REG_POSITION_MID
 *
 * @return Midpoint value converted by SERVO_U16_TO_ANGLE().
 * @warning Marked DO NOT USE / "does not actually work as intended" in your notes.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
float check_servo_midpoint (void) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return -1;
	}

	uint16_t raw_servo_midpoint = receive_can_msg(REG_POSITION_MID);

	return SERVO_U16_TO_ANGLE(raw_servo_midpoint);
}



// ============================================================================
// Velocity & Acceleration
// ============================================================================

/**
 * @brief Set the acceleration time of the servo (speed-up ramp time).
 *
 * Register written: REG_SPEED_UP
 * Units (per your comment): milliseconds.
 *
 * @param servo_speed_up_time Time in ms (sent as raw 16-bit value by send_can_msg()).
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_speed_up (float servo_speed_up_time) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_speed_up_time, REG_SPEED_UP);

	return;
}

/**
 * @brief Set the deceleration time of the servo (speed-down ramp time).
 *
 * Register written: REG_SPEED_DN
 * Units (per your comment): milliseconds.
 *
 * @param servo_speed_down_time Time in ms (sent as raw 16-bit value by send_can_msg()).
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_speed_down (float servo_speed_down_time) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_speed_down_time, REG_SPEED_DN);

	return;
}

/**
 * @brief Set the maximum speed value that operates in a normal state.
 *
 * Register written: REG_VELOCITY_MAX
 *
 * @param servo_max_velocity Raw max-velocity value.
 * @note Your comment says valid range is 0..4095, but no clamp is applied.
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_max_velocity (float servo_max_velocity) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_max_velocity, REG_VELOCITY_MAX);

	return;
}



// ============================================================================
// Servo temperature handling
// ============================================================================

/**
 * @brief Returns the temperature of the servo's internals.
 *
 * Register read: REG_TEMP
 *
 * Conversion used in this code:
 *  - temp_degC = 175.72 * raw/65536 - 46.85
 *
 * @return Temperature in degrees Celsius.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
float check_servo_temp (void) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return -1;
	}

	uint16_t raw_servo_temperature = receive_can_msg(REG_TEMP);

	uint16_t servo_temperature = 175.72 * raw_servo_temperature / 65536 - 46.85; // in degrees

	return servo_temperature;
}

/**
 * @brief Set a temperature max limit for the MCU temperature.
 *
 * Register written: REG_TEMPER_MAX
 *
 * @param servo_max_temperature Max temperature limit (raw units per servo firmware).
 * @note Your comment: "Servo goes into emergency mode" when exceeded.
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_max_temperature (float servo_max_temperature) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_max_temperature, REG_TEMPER_MAX);

	return;
}

/**
 * @brief Set a temperature min limit for the MCU temperature.
 *
 * Register written: REG_TEMPER_MIN
 *
 * @param servo_min_temperature Min temperature limit (raw units per servo firmware).
 * @note Your comment: "Servo goes into emergency mode" when undercut.
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_min_temperature (float servo_min_temperature) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_min_temperature, REG_TEMPER_MIN);

	return;
}



// ============================================================================
// Servo current and voltage handling
// ============================================================================

/**
 * @brief Set the servo max current draw.
 *
 * Register written: REG_CURRENT_MAX
 *
 * @param max_servo_current Max current limit (mA per your comment).
 * @note Your comment suggests range: 1mA..6000mA (stall current ~6000mA).
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_max_current (float max_servo_current) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(max_servo_current, REG_CURRENT_MAX);

	return;
}

/**
 * @brief Returns the current draw of the servo.
 *
 * Register read: REG_CURRENT
 *
 * @return Raw current value (units depend on servo firmware/datasheet).
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
float check_servo_current (void) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return -1;
	}

	uint16_t servo_current = receive_can_msg(REG_CURRENT);


	return servo_current;
}

/**
 * @brief Returns the configured max-rated stall current of the servo.
 *
 * Register read: REG_CURRENT_MAX
 *
 * @return Raw max current value (units depend on servo firmware/datasheet).
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
float check_servo_max_current (void) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return -1;
	}

	uint16_t max_servo_current = receive_can_msg(REG_CURRENT_MAX);


	return max_servo_current;
}

/**
 * @brief Set the maximum supply voltage limit of the servo.
 *
 * Register written: REG_VOLTAGE_MAX
 *
 * @param servo_max_voltage Max voltage limit (raw units per servo firmware/datasheet).
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_max_voltage (float servo_max_voltage) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_max_voltage, REG_VOLTAGE_MAX);

	return;
}


/**
 * @brief Set the minimum supply voltage limit of the servo.
 *
 * Register written: REG_VOLTAGE_MIN
 *
 * @param servo_min_voltage Min voltage limit (raw units per servo firmware/datasheet).
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_min_voltage (float servo_min_voltage) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_min_voltage, REG_VOLTAGE_MIN);

	return;
}



// ============================================================================
// Servo End Limits
// ============================================================================

/**
 * @brief Set a soft limit for the servo's max position.
 *
 * Register written: REG_POSITION_MAX_LIMIT
 * @param servo_position_max_limit Soft max limit (raw units per servo encoding).
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_max_position_limit (float servo_position_max_limit) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_position_max_limit, REG_POSITION_MAX_LIMIT);

	return;
}

/**
 * @brief Set a soft limit for the servo's min position.
 *
 * Register written: REG_POSITION_MIN_LIMIT
 * @param servo_position_min_limit Soft min limit (raw units per servo encoding).
 * @note Function accepts float but sends uint16_t; values will be truncated.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_min_position_limit (float servo_position_min_limit) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_position_min_limit, REG_POSITION_MIN_LIMIT);

	return;
}

/**
 * @brief Set a hard limit for servo's max position.
 *
 * Register written: REG_POS_MAX
 *
 * @param servo_position_max Hard max position (raw units per servo encoding).
 *
 * @warning Your comment: this puts the servo into emergency stop mode and may be hard to exit.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_max_position (float servo_position_max) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_position_max, REG_POS_MAX);

	return;
}

/**
 * @brief Set a hard limit for servo's min position.
 *
 * Register written: REG_POS_MIN
 *
 * @param servo_position_min Hard min position (raw units per servo encoding).
 *
 * @warning Your comment: this puts the servo into emergency stop mode and may be hard to exit.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void set_servo_min_position (float servo_position_min) {

	if (!SERVO_OK()) {
		servo_error_handler();
		return;
	}

	send_can_msg(servo_position_min, REG_POS_MIN);

	return;
}



// ============================================================================
// Servo Status Handling
// ============================================================================

/**
 * @brief Check whether the servo is in a normal (non-emergency) state.
 *
 * Register read: REG_EMERGENCY_STOP (your note indicates address ~0x48).
 *
 * This function checks servo_status bitmasks:
 *  - ERR_POS_MIN / ERR_POS_MAX
 *  - ERR_MCU_TEMP_UND / ERR_MCU_TEMP_OVR
 *  - ERR_VOLT_UND / ERR_VOLT_OVR
 *
 * @return true if no checked emergency bits are set, false otherwise.
 *
 * @warning If receive_can_msg() fails, it can return 0xFFFF/0xFFFE (because it returns uint16_t).
 *          That may cause this function to report many errors and return false.
 */
bool SERVO_OK (void) {

    uint16_t servo_status = receive_can_msg(REG_EMERGENCY_STOP);  // <-- should be 0x48
    bool servo_condition = true;

    if (servo_status & ERR_POS_MIN) {
        servo_condition = false;
    }

    if (servo_status & ERR_POS_MAX) {
        servo_condition = false;
    }

    if (servo_status & ERR_MCU_TEMP_UND) {
        servo_condition = false;
    }

    if (servo_status & ERR_MCU_TEMP_OVR) {
        servo_condition = false;
    }

    if (servo_status & ERR_VOLT_UND) {
        servo_condition = false;
    }

    if (servo_status & ERR_VOLT_OVR) {
        servo_condition = false;
    }

    return servo_condition;  // no emergency bits set
}

/**
 * @brief Perform servo fault recovery sequence.
 *
 * This function attempts to restore normal operation after
 * SERVO_OK() reports a fault condition.
 *
 * Recovery actions:
 *  - Enable forced emergency speed-down mode via REG_POWER_CONFIG.
 *  - Issue a software reset request via REG_POWER_CONFIG.
 *  - Clear forced emergency-stop mode via REG_POWER_CONFIG.
 *  - Reapply default configuration via servo_init().
 *
 * Registers affected:
 *  - REG_POWER_CONFIG (forced ES mode, software reset)
 *  - REG_POSITION_MAX_LIMIT / REG_POSITION_MIN_LIMIT
 *  - REG_VOLTAGE_MAX / REG_VOLTAGE_MIN
 *  - REG_CURRENT_MAX
 *
 * @note This function assumes the underlying fault condition
 *       (e.g., voltage or temperature) has been resolved.
 */
void servo_error_handler (void) {

		// Stay safe first
	    servo_set_speed_down_emergeny_mode();

	    // Try to recover
	    servo_soft_reset_only();
	    HAL_Delay(100); // give servo time to reboot

	    // If still faulted, DO NOT clear emergency mode.
	    servo_init();

	    if (!SERVO_OK()) {
	        return; // try again next time
	    }

	    // Now it is safe to leave emergency mode
	    servo_clear_forced_es();
	    HAL_Delay(50);

	    return;
}



/**
 * @brief Force the servo into "Speed_Down" behavior in forced emergency mode.
 *
 * Register written: REG_POWER_CONFIG
 * Field: Forced ES mode (typically bits [10:9] per your earlier screenshot/notes)
 * Value: PWR_FORCED_ES_SPEED_DOWN
 *
 * @note This function writes only the forced ES mode bits (as provided by the macro),
 *       leaving other reserved bits as 0.
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void servo_set_speed_down_emergeny_mode(void) {

	if (!SERVO_OK()) {
		return;
	}

    // Only touch allowed bits; keep reserved as 0
    send_can_msg((uint16_t)PWR_FORCED_ES_SPEED_DOWN, REG_POWER_CONFIG);
}

/**
 * @brief Clear forced emergency-stop mode (set forced ES mode to OFF).
 *
 * Register written: REG_POWER_CONFIG
 * Value: PWR_FORCED_ES_OFF
 *
 * @warning If SERVO_OK() is false, this function resets the MCU.
 */
void servo_clear_forced_es(void) {

    send_can_msg((uint16_t)PWR_FORCED_ES_OFF, REG_POWER_CONFIG);
}

/**
 * @brief Reset the servo over CAN (software reset request).
 *
 * Register written: REG_POWER_CONFIG
 * Value: PWR_SW_RESET_BIT (typically bit0)
 *
 * @note Your comment: this may implicitly clear forced ES mode since REG_POWER_CONFIG is overwritten.
 * @warning No delay/retry logic is implemented here (caller should throttle).
 */
void servo_soft_reset_only(void) {
    // This may implicitly clear forced ES mode since you're overwriting the reg
    send_can_msg((uint16_t)PWR_SW_RESET_BIT, REG_POWER_CONFIG);
}
