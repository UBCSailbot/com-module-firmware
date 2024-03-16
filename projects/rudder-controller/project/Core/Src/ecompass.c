/*
 * ecompass.c
 *
 *  Created on: Feb 10, 2024
 *      Author: Moiz
 */
#include "main.h"
#include "ecompass.h"


/**
 *  Name:       ecompass_wr
 *
 *
 *  Purpose:    write a byte to encompass register
 *
 *  Params:     reg_addr - register address to write into
 *  			value - value to be written into register
 *
 *  Return:     HAL_StatusTypeDef
 *
 *  Notes:      none
 */
HAL_StatusTypeDef ecompass_wr(uint8_t reg_addr, uint8_t value){
	return HAL_I2C_Mem_Write(&hi2c2, EC_ADDR, reg_addr, sizeof(reg_addr), &value, sizeof(uint8_t), 100);
}

/**
 *  Name:       ecompass_rd
 *
 *
 *  Purpose:    Read a byte to encompass register
 *
 *  Params:     reg_addr - register address to read from
 *  			value - byte ptr to store and return read data
 *
 *  Return:     HAL_StatusTypeDef
 *
 *  Notes:      none
 */
HAL_StatusTypeDef ecompass_rd(uint8_t reg_addr, uint8_t* value) {
	return HAL_I2C_Mem_Read(&hi2c2, EC_ADDR, reg_addr, sizeof(reg_addr), value, sizeof(uint8_t), 100);
}

/**
 *  Name:       ecompass_wr_16b
 *
 *
 *  Purpose:    Write upper and lower bytes into separate registers
 *
 *  Params:     reg_addr_upper - register containing upper byte
 *  			reg_addr_lower - register containing lower byte
 *  			value - 16bit value to be written into register
 *
 *  Return:     HAL_StatusTypeDef
 *
 *  Notes:      Return value can only be used to check 'HAL_OK'
 */
HAL_StatusTypeDef ecompass_wr_16b(uint8_t reg_addr_upper, uint8_t reg_addr_lower, uint16_t value) {
	HAL_StatusTypeDef status1, status2;
	uint8_t upper_val = (value & 0xFF00) >> 8;
	uint8_t lower_val = value & 0x00FF;

	status1 = ecompass_wr(reg_addr_upper, upper_val);
	status2 = ecompass_wr(reg_addr_lower, lower_val);

	// TODO: Could do better error handling. For now return error if either one is error.
	return status1 | status2;
}

/**
 *  Name:       ecompass_rd_16b
 *
 *
 *  Purpose:    Read upper and lower bytes from separate registers
 *
 *  Params:     reg_addr_upper - register containing upper byte
 *  			reg_addr_lower - register containing lower byte
 *  			value - 16bit ptr to store and return read data
 *
 *  Return:     HAL_StatusTypeDef
 *
 *  Notes:      Return value can only be used to check 'HAL_OK'
 */
HAL_StatusTypeDef ecompass_rd_16b(uint8_t reg_addr_upper, uint8_t reg_addr_lower, uint16_t* value) {
	HAL_StatusTypeDef status1, status2;
	uint8_t upper_val, lower_val;

	status1 = ecompass_rd(reg_addr_upper, &upper_val);
	status2 = ecompass_rd(reg_addr_lower, &lower_val);

	*value = ((uint16_t)upper_val) << 8 | lower_val;

	// TODO: Could do better error handling. For now return error if either one is error.
	return status1 | status2;
}

/**
 *  Name:       ecompass_init
 *
 *
 *  Purpose:    none
 *
 *  Params:     none
 *
 *  Return:     EC_StatusTypeDef
 *
 *  Notes:      none
 */
EC_StatusTypeDef ecompass_init(void) {
	// TODO: Check Calibration state - reg 0x1E
	return EC_OK;
}

/**
 *  Name:       ecompass_isCalibrated
 *
 *
 *  Purpose:    Returns the calibration status of specified register
 *
 *  Params:     cal_name - EC_CalibratonName
 *
 *  Return:     EC_StatusTypeDef
 *
 *  Notes:      none
 */
EC_StatusTypeDef ecompass_isCalibrated(EC_CalibratonName cal_name) {
	HAL_StatusTypeDef status;
	uint8_t mask = 0;
	const uint8_t twobits = 3;
	uint8_t calibratoin_result = 0;

	mask = twobits << (uint8_t) cal_name;

	status = ecompass_rd(EC_REG_CALIBRATION, &calibratoin_result);
	if (status != HAL_OK) return EC_ERROR_READ;

	return (calibratoin_result & mask) ? EC_CALIBRATED : EC_UNCALIBRATED;
}


//-- Getters --

EC_StatusTypeDef ecompass_getBearing(uint16_t* bearing) {
	HAL_StatusTypeDef status;

	status = ecompass_rd_16b(EC_REG_BEARING_UPPER, EC_REG_BEARING_LOWER, bearing);
	if (status != HAL_OK) return EC_ERROR_READ;

    return EC_OK;
}

EC_StatusTypeDef ecompass_getPitch(uint8_t* pitch) {
	HAL_StatusTypeDef status;

	status = ecompass_rd(EC_REG_PITCH, pitch);
	if (status != HAL_OK) return EC_ERROR_READ;

    return EC_OK;
}

EC_StatusTypeDef ecompass_getRoll(uint8_t* roll) {
	HAL_StatusTypeDef status;

	status = ecompass_rd(EC_REG_ROLL, roll);
	if (status != HAL_OK) return EC_ERROR_READ;

    return EC_OK;
}

EC_StatusTypeDef ecompass_getTemp(uint16_t* temp) {
	HAL_StatusTypeDef status;

	status = ecompass_rd_16b(EC_REG_TEMP_UPPER, EC_REG_TEMP_LOWER, temp);
	if (status != HAL_OK) return EC_ERROR_READ;

    return EC_OK;
}
