/**
 * Description: For anything that operates the board physcially.
 *
 * Author: Jordan, Matthew, Peter
 *
 * Note: None
 * */

/* Includes ------------------------------------------------------------------*/
#include "bord.h"
#include "comp.h"
#include "numb.h"
#include "oprt.h"

/* Variables ------------------------------------------------------------------*/
uint16_t ambient;
uint8_t UART1_rxBuffer[1] = {0};

/* Functions ------------------------------------------------------------------*/
int veml3328_start(void){
	while (veml3328_init() == 0) return 0;

	veml3328_config(0000);
	pwm1_init_ch1(50);
	ambient = avg_amb();

	return 1;
}

void veml3328_oprt(void){
	veml3328_rd_rgb();
	if (g_data > ambient+5) pwm1_set_ch1(90);
	if (g_data < ambient-5) pwm1_set_ch1(10);
}

void gpio(void){
	if (gpio_rd_e2() == GPIO_PIN_SET) gpio_wr_e3();
	else gpio_rs_e3();
	if (gpio_rd_e4() == GPIO_PIN_SET) gpio_wr_e5();
	else gpio_rs_e5();
}

/* Debugging ------------------------------------------------------------------*/

/* Keyboard Mapping:
 * Takes in input from the serial monitor and returns ASCII int value.
 * Refer to the table: https://www.cs.cmu.edu/~pattis/15-1XX/common/handouts/ascii.html
 */
int debug_key(void){
	pwm3_init_ch1(50);

	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);

	return *UART1_rxBuffer;
}



