/**
 * Description: For computational things that are required for the board's operation.
 *
 * Author: Jordan, Matthew, Peter
 *
 * Note: None
 * */

/* Includes ------------------------------------------------------------------*/
#include "bord.h"
#include "comp.h"
#include "numb.h"

/* Variables ------------------------------------------------------------------*/
int avg = 0;
int n = 0;

/* Functions ------------------------------------------------------------------*/
uint16_t avg_amb(void) {
	for (n = 0; n < 50; n++){
		veml3328_rd_rgb();
		avg += g_data;
	}

	return avg/50;
}

