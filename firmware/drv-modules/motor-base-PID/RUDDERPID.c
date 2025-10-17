/*
 * Motor_PID.h
 *
 *  Created on: Mar 18, 2025
 *      Author: Chukwudalu Joshua Obi
 */

 #include "Motor_PID.h"
 #include <stdint.h>
 #include <math.h>
 #include <stdio.h>

void DAC_STEP(int step) {

	/*
		 * Adjusts the DAC output in steps of 1/50th - used for testing the minimum power required to drive
		 * certain loads attached to the motor
		 *
		 * @ params
		 *  step - an int value between 0 and 50 e.g. passing 2 means 2/50 output of the DAC
		 *
		 */

    uint32_t Motor_DAC = (uint32_t)(i * 4095.0f / 50); // Keep DAC value as 12-bit resolution
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Motor_DAC); // Set DAC output
    float voltage = (i * 3.3f) /100; // Convert DAC value to voltage

    printf("%u\r\n", Motor_DAC);
    printf("V: %d.%03d V\r\n", (int)voltage, (int)(fabsf(voltage * 1000)) % 1000);
}


void Set_Motor(float_t Motor_Control)
{

	/*
	 * Sets DAC value between 0 and 4095 depending on magnitude of Motor_Control to change speed
	 * where the DAC value is scaled by a float in range of 0 and 1
	 *
	 * @ params
	 *  Motor_Control - a float value passed between 0 and 1 - 1 indicates full output (max speed) and 0 indicates
	 *  				no output (the motor stops moving)
	 */


    uint32_t Motor_DAC = (uint32_t)(fabsf(Motor_Control) * 4095.0f);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Motor_DAC); // Manually sets the output of the DAC


    // Sets PA7 high if Motor_Control is positive, otherwise low

    if (Motor_Control < 0.0f)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    }

    return;
}


void PI_Motor(int32_t desired_heading, int32_t current_heading,
              float* integral_error, uint32_t* last_time_stamp,
              int32_t* past_encoder_heading, int8_t* past_motor_direction)
{

	/*
	 * Calculates the error in boat's heading and adjusts the rudder accordingly
	 *
	 * @params
	 * desired_heading - target rudder angle (sent by sailing conditions state machine) - E.D.
	 * current_heading - current rudder angle (sent by encoder) - C.C.
	 * integral_error - sums up past errors for PI control
	 * last_time_stamp - saves the last times the PI_controller was called to calculate change in time
     * past_encoder_heading - stores the previous encoder reading to check if the motor is still moving
     * past_motor_direction - stores last motor direction to prevent immediate reversals
	 */


		uint32_t current_time_stamp = HAL_GetTick(); // Returns number of milliseconds since startup

	// Uncomment the below statements for debug print statements

	/*	printf("CH: %li\r\n",current_heading);
		printf("DH: %li\r\n",desired_heading);
	*/

	// Unsigned casting arithmetic handles if timer overflows - Divide by 1000 to convert from ms to s
		float del_time = (current_time_stamp - *last_time_stamp)/1000.0f;


	// Handle event if del_time = 0 - handle zero division error
		if (del_time < 0.001f) {
		    del_time = 0.001f; // Minimum integration time step
		}

		int32_t error = desired_heading - current_heading;
		printf("%li\r\n",error);
		int8_t direction = (error > 0) - (error < 0); // Determine direction (-1, 0, or 1)
		float Angular_Velocity = (current_heading-*past_encoder_heading)/del_time;


    // Check if error is within the acceptable threshold
		if (fabsf(error) < ERROR_THRESHOLD)
		{
			Set_Motor(0); // Stop the motor
			*integral_error = 0.0f; // Reset the integral term
			*last_time_stamp = HAL_GetTick(); // Update timestamp
			*past_encoder_heading = current_heading; // Store last encoder value
			// *past_motor_direction = 0; // Mark motor as stopped
			//printf("No err.\r\n");
			return;
		}


    // Ensure motor is fully stopped before allowing direction change
		if ((Angular_Velocity > 0 && direction < 0) || (Angular_Velocity < 0 && direction > 0)){
			Set_Motor(0);
			*integral_error = 0.0f; // Reset the integral term
			*last_time_stamp = HAL_GetTick(); // Update timestamp
			*past_encoder_heading = current_heading; // Store last encoder value
			return;
		}


    // Routine for when error is greater than set threshold or if the direction has not been changed


	// Update the integral term
		*integral_error += (float)error * (float)del_time;
		*integral_error = fminf(fmaxf(*integral_error, -INTEGRAL_LIMIT), INTEGRAL_LIMIT); // Clamp the integral term

	// Compute the PI controller output
		float motor_output =  (PROPORTIONAL_GAIN * error) + (INTEGRAL_GAIN * (*integral_error));

	// Sets the motor to minimum power for movement if in dead zone - temporary value for the no-load motor case
		if (fabsf(motor_output) < 0.06){
			motor_output = direction * 0.06;
		}

		motor_output = fminf(fmaxf(motor_output, -MAX_MOTOR), MAX_MOTOR); // clamp the motor output if it's greater than 1.0

	// Uncomment the below statements for debug print statements

	/*
		printf("Dir: %li\r\n",direction);
		printf("PG: %li\r\n",PROPORTIONAL_GAIN);
		printf("IG: %li\r\n",INTEGRAL_GAIN);
		printf("Err: %li\r\n",error);
		printf("t: %d.%03d\r\n", (int)del_time, (int)(fabsf(del_time*1000)) % 1000);
		printf("IE: %d.%03d\r\n", (int)*integral_error, (int)(fabsf(*integral_error * 1000)) % 1000);
		printf("MO: %d.%03d\r\n", (int)motor_output, (int)(fabsf(motor_output * 1000)) % 1000);
		printf("\r\n");

	 */


		Set_Motor(motor_output); // Actuate the motor

		*last_time_stamp = current_time_stamp; // Update the timestamp for the next iteration
		*past_encoder_heading = current_heading; // Store the last encoder reading
		*past_motor_direction = direction; // Store the last applied direction

    return;

}