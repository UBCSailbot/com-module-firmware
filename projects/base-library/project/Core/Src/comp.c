/**
 * Description: For things that are directly connected to the pins on the board.
 *
 * Author: Jordan, Matthew, Peter
 *
 * Note: None
 * */

/* Includes ------------------------------------------------------------------*/
#include "bord.h"
#include "comp.h"

/* Variables ------------------------------------------------------------------*/
HAL_StatusTypeDef r,g,b;
uint16_t conf, r_data, g_data, b_data;

/* Functions ------------------------------------------------------------------*/
/* veml3328 initialization*/
HAL_StatusTypeDef veml3328_init(void) {

	HAL_StatusTypeDef stat1, deviceId;
	uint16_t regVal;

	// Check register device ID (should be 0x28)
	stat1 = i2c_rd(hi2c1, veml3328_addr, veml3328_reg_deviceID, &regVal);
	if (stat1 != HAL_OK) return 0;

	deviceId = (uint8_t)(regVal & 0xFF);
	if (deviceId != 0x28) return 0;

	return 1;
}

/* veml3328 configuration */
void veml3328_config(uint16_t mode){
	i2c_wr(hi2c1, veml3328_addr, veml3328__conf, mode); // change 0000 to change the sensor config.
}

/* veml3328 read R,G,B */
void veml3328_rd_rgb(void){
    r = i2c_rd(hi2c1, veml3328_addr, R_, &r_data);
    g = i2c_rd(hi2c1, veml3328_addr, G_, &g_data);
    b = i2c_rd(hi2c1, veml3328_addr, B_, &b_data);
}



