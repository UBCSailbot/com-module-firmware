

/*
 * encoder.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Jasia
 */

#include "encoder.h"

EN_StatusTypeDef encoder_init(void){ //does configurations
	printf("encoder init\n\r");
	return EN_OK;
}

EN_StatusTypeDef encoder_callibrate(void) {
	printf("callibrate encoder\n\r");
	return EN_OK;
}

EN_StatusTypeDef encoder_read(uint16_t* rawRead) {
	printf("read encoder\n\r");
	return EN_OK;
}

EN_StatusTypeDef encoder_getPosition(uint16_t* position) {
	printf("get position encoder\n\r");
	return EN_OK;
}

uint8_t encoder_is_reached_max(void) {
	printf("encoder reached max");
	return EN_OK;
}

