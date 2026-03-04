/*
 *	veml3328.c
 *
 *  Created on: Mar 29, 2024
 *	Author: Peter
 */

/* Includes ------------------------------------------------------------------*/
#include "veml3328.h"

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

	i2c_wr(hi2c1, veml3328_addr, veml3328__conf, 0000); // 0000 is the default configuration.

	return 1;
}

/* veml3328 read R,G,B */
void veml3328_rd_rgb(void){
    r = i2c_rd(hi2c1, veml3328_addr, R_, &r_data);
    g = i2c_rd(hi2c1, veml3328_addr, G_, &g_data);
    b = i2c_rd(hi2c1, veml3328_addr, B_, &b_data);
}

uint16_t veml3328_avg_amb(void){
	int n = 0;
	int avg = 0;

	for (n = 0; n < 50; n++){
		veml3328_rd_rgb();
		avg += g_data;
	}

	return avg/50;
}

int veml3328_run(uint16_t amb){
	veml3328_rd_rgb();
	printf("ambient is: %u  data is: %u\r\n", amb, g_data);

	if (g_data > amb+5) return 90;
	if (g_data < amb-5) return 10;

	return 5;
}











