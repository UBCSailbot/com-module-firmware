/*
 * controls.c
 *
 *  Created on: Mar 16, 2024
 *      Author: Jasia
 */
#include "controls.h"
#include "ecompass.h"
#include "motor.h"
#include "encoder.h"

#include <stdlib.h>


//future: turn it into variable so SOFT can send values
#define KI 0.075
#define KP 0.011
#define KD 1.022

#define ADC_MAX 255
#define ADC_MIN -255

#define SMALL_ANGLE 2

#define ERROR_MAX 100
#define ERROR_MIN -100

#define DELAY_TIME 100

#define FULL_ROTATION 360
#define HALF_ROTATION 180

//REMOVE THIS WHEN COMs has their actual implementation
double get_desired_heading(void){
	return 0.0;
}


void main_ctrl_loop(void){
	motor_init();
	ecompass_init();
	encoder_init();

	while (1){
		//add watchdog timer here
		run_pid();
		HAL_Delay(DELAY_TIME);
	}
}

void run_pid(void){
	//change to double
	static int sum_errors = 0, prev_error = 0;
	int P_term, I_term, D_term;
	int error, adjusted_error, current_error;
	int PWM_value, PWM_temp = 100; //gotta change this
	uint16_t bearing;


	if (ecompass_getBearing(&bearing) != EC_OK) {
		return;
		//figure out further error stuff
	}

	error = get_desired_heading() - bearing;

	adjusted_error = error;


	//dead zone case (small angle error shouldn't lead to pwm change) --> avoids small constant corrections
	/*if (abs(error) < SMALL_ANGLE) {
		error = 0; //does that make sense?
		adjusted_error = 0;
		return; //stops PID
	}
	//COME BACK TO THIS- HAVE TO CONSIDER THE CASE WHERE WE TRIGGER PID
	*/

	//adjusting error values
	if (error > HALF_ROTATION) {adjusted_error = error - FULL_ROTATION;} //so it now moves port not stbd, though error is +
	if (error < -HALF_ROTATION) {adjusted_error = error + FULL_ROTATION;} //so it now moves stbd nor port, though error is -


	current_error = abs(adjusted_error); //does it have to be the absolute?

	P_term = KP * current_error; //calculate p term

	//make sure we dont have integral windup (for when we have sudden change in current/desired heading)
	//since we'll be using error polarity to determine which way to move, does it make sense to not change error var
	//directly? we can use a different param just for calculations? i changed eror to adjusted_error here

	if (adjusted_error >= ERROR_MAX) {
		adjusted_error = ERROR_MAX;
	}

	else if (adjusted_error <= ERROR_MIN) {
		adjusted_error = ERROR_MIN;
	}

	sum_errors += adjusted_error; //accumulated error over time, used for Pi term calculation
    I_term = KI * sum_errors; //calculate i term

	//calculate d term from the differential between previous error and current error
    D_term = KD * (current_error - prev_error);
    prev_error = current_error;


    //calculate PWM output value based on previous PWM value
	PWM_value = PWM_temp - (P_term + I_term + D_term);

	//PWM overvoltage protection***
	if (PWM_value > ADC_MAX) {
		PWM_value = ADC_MAX;
	}

	else if (PWM_value < ADC_MIN) {
		PWM_value = ADC_MIN;
	}

//des_heading > curr_heading → error > 0 → increase curr_heading to reduce error --> Move cw i.e stbd
	if (adjusted_error > 0) {
		motor_move_stbd(); //determines direction (pin)
	}

//des_heading < curr_heading → error < 0 → decrease curr_heading to increase error → move ccw i.e. move port
	else if (adjusted_error < 0) {
		motor_move_port();
	}


	//sends the pwm to motor
	motor_set_duty_cycle(PWM_value);

	PWM_temp = PWM_value;

}


