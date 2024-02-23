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

/* Functions ------------------------------------------------------------------*/
int demo(void) {
	//uint8_t tx_buffer[30] = "uart test\r\n";

	while (veml3328_init() == 0) return 0;

	veml3328_config(0000);
	pwm_init_ch1(50);
	ambient = avg_amb();
	HAL_Delay(10);

	printf("printf test\r\n");
	//uart_wr(huart1, tx_buffer);

	while (1) {
	  veml3328_rd_rgb();
	  if (g_data > ambient+5) {pwm_set_ch1(10);}
	  if (g_data < ambient-5) {pwm_set_ch1(90);}

	  if (gpio_rd_e2() == GPIO_PIN_SET) gpio_wr_e3();
	  else {gpio_rs_e3();}
	  if (gpio_rd_e4() == GPIO_PIN_SET){
		  gpio_wr_e5();

	  }
	  else {gpio_rs_e5();}
	  HAL_Delay(10);
	}

	return 1;
}
