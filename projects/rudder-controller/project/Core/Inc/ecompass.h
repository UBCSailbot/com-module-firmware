/*
 * ecompass.h
 *
 *  Created on: Feb 10, 2024
 *      Author: Jasia
 */

#ifndef SRC_ECOMPASS_H_
#define SRC_ECOMPASS_H_

#include "main.h"
#include <stddef.h>

extern I2C_HandleTypeDef hi2c2;

// Ecompass Address
#define EC_ADDR											0xC0

// Data Registers
#define EC_REG_SOFTVERSION 	  		                    0x00
#define EC_REG_BEARING_UPPER							0x02
#define EC_REG_BEARING_LOWER							0x03
#define EC_REG_PITCH									0x04
#define EC_REG_ROLL										0x05
#define EC_REG_TEMP_UPPER								0x18
#define EC_REG_TEMP_LOWER								0x19
#define EC_REG_CALIBRATION								0x1E

typedef enum
{
	EC_OK = 0x00,
	EC_ERROR = 0x01,
	EC_ERROR_READ = 0x02,
	EC_ERROR_WRITE = 0x03,
	EC_CALIBRATED = 0x04,
	EC_UNCALIBRATED = 0x05
} EC_StatusTypeDef;

typedef enum
{
	EC_BEARING_8B,
	EC_BEARING_16B,
	EC_PTCH,
	EC_REG_TEMP,
	EC_CALIBRATION
} EC_RegName;

typedef enum
{
	EC_CAL_MAGNETO = 0x00,
	EC_CAL_ACCEL = 0x02,
	EC_CAL_GYRO = 0x04,
	EC_CAL_SYSTEM = 0x06
} EC_CalibratonName;


HAL_StatusTypeDef ecompass_wr(uint8_t reg_addr, uint8_t value);
HAL_StatusTypeDef ecompass_rd(uint8_t reg_addr, uint8_t* value);
HAL_StatusTypeDef ecompass_wr_16b(uint8_t reg_addr_upper, uint8_t reg_addr_lower, uint16_t value);
HAL_StatusTypeDef ecompass_rd_16b(uint8_t reg_addr_upper, uint8_t reg_addr_lower, uint16_t* value);
EC_StatusTypeDef ecompass_init(void);
EC_StatusTypeDef ecompass_isCalibrated(EC_CalibratonName cal_name);

EC_StatusTypeDef ecompass_getBearing(uint16_t* bearing);
EC_StatusTypeDef ecompass_getPitch(uint8_t* pitch);
EC_StatusTypeDef ecompass_getRoll(uint8_t* roll);
EC_StatusTypeDef ecompass_getTemp(uint16_t* temp);

#endif /* SRC_ECOMPASS_H_ */


