/*
 * Motor_PID.h
 *
 *  Created on: Mar 18, 2025
 *      Author: Chukwudalu Joshua Obi
 */

 #include "RUDDERPID.h"
 #include <stdint.h>
 #include <math.h>
 #include <stdio.h>

typedef struct {
	uint32_t currentDirectionState;
	float integralError;
	uint32_t pastEncoderTimeStamp;
	float pastEncoderValue;
	uint8_t motorEnable;
 } MOTOR_DYNAMIC_INFO;

MOTOR_CONFIG globalMCfg;
MOTOR_DYNAMIC_INFO globalMDyn = {
	.currentDirectionState = 0,
	.integralError = 0,
	.pastEncoderTimeStamp = 0,
	.pastEncoderValue = 0,
	.motorEnable = 0,
};



void Setup_Motor(MOTOR_CONFIG motorConfig){
	globalMCfg = motorConfig;
	HAL_DAC_Start(globalMCfg.motorDacPeripheral, globalMCfg.motorDacChannel);
	HAL_GPIO_WritePin(globalMCfg.reverseGPIOPeripheral, globalMCfg.reverseGPIOPin, GPIO_PIN_RESET);
}

// fix this
void DAC_STEP(int step) {

	/*
	 * Adjusts the DAC output in steps of 1/50th - used for testing the minimum power required to drive
	 * certain loads attached to the motor
	 *
	 * @ params
	 *  step - an int value between 0 and 50 e.g. passing 2 means 2/50 output of the DAC
	 *
	 */

    uint8_t Motor_DAC = (uint8_t)(step * 4095.0f / 50); // Keep DAC value as 12-bit resolution
    HAL_DAC_SetValue(globalMCfg.motorDacPeripheral, globalMCfg.motorDacChannel, DAC_ALIGN_12B_R, Motor_DAC); // Set DAC output
    float voltage = (step * 3.3f) / 50; // Convert DAC value to voltage

    printf("%u\r\n", Motor_DAC);
    printf("V: %d.%03d V\r\n", (int)voltage, (int)(fabsf(voltage * 1000)) % 1000);
}


void Set_Motor_Calibrated(float_t Motor_Control){
	if(fabs(Motor_Control) < MIN_MOTOR_VELOCITY_COMMAND){
		Set_Motor_Raw(0);
		return;
	}

	if(Motor_Control > 0){
		Set_Motor_Raw(Motor_Control * (1 - MIN_MOTOR) + MIN_MOTOR);
	} else {
		Set_Motor_Raw(Motor_Control * (1 - MIN_MOTOR) - MIN_MOTOR);
	}

}
void Enable_Motor(){
	globalMDyn.motorEnable = 1;
	HAL_GPIO_WritePin(globalMCfg.enableGPIOPeripheral, globalMCfg.enableGPIOPin, GPIO_PIN_SET);
}

void Disable_Motor(){
	globalMDyn.motorEnable = 0;
	HAL_GPIO_WritePin(globalMCfg.enableGPIOPeripheral, globalMCfg.enableGPIOPin, GPIO_PIN_RESET);
	HAL_DAC_SetValue(globalMCfg.motorDacPeripheral, globalMCfg.motorDacChannel, DAC_ALIGN_12B_R, 0);

}

void Set_Motor_Raw(float_t Motor_Control)
{

	/*
	 * Sets DAC value between 0 and 4095 depending on magnitude of Motor_Control to change speed
	 * where the DAC value is scaled by a float in range of 0 and 1
	 *
	 * @ params
	 *  Motor_Control - a float value passed between 0 and 1 - 1 indicates full output (max speed) and 0 indicates
	 *  				no output (the motor stops moving)
	 */

	if(globalMDyn.motorEnable == 0)
			return;
	if(fabs(Motor_Control) < MIN_MOTOR_VELOCITY_COMMAND){
	    HAL_DAC_SetValue(globalMCfg.motorDacPeripheral, globalMCfg.motorDacChannel, DAC_ALIGN_12B_R, 0);
	    return;
	}

    if (Motor_Control < 0.0f)
    {
//        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(globalMCfg.reverseGPIOPeripheral, globalMCfg.reverseGPIOPin, GPIO_PIN_RESET);
    }
    else if (Motor_Control > 0.0f)
    {
        HAL_GPIO_WritePin(globalMCfg.reverseGPIOPeripheral, globalMCfg.reverseGPIOPin, GPIO_PIN_SET);
    }

    uint32_t Motor_DAC = (uint32_t)(fabsf(Motor_Control) * 4095.0f);
    Motor_DAC = fmin(DAC_MAX_OUTPUT, Motor_DAC);

    HAL_DAC_SetValue(globalMCfg.motorDacPeripheral, globalMCfg.motorDacChannel, DAC_ALIGN_12B_R, Motor_DAC); // Manually sets the output of the DAC


    // Sets PA7 high if Motor_Control is positive, otherwise low



    return;
}


void PI_Motor(float desired_heading, float current_heading, uint32_t angle_timestamp)
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
     */
	if (current_heading > 180.0 || current_heading < -180.0)
		return;

	if(globalMDyn.motorEnable == 0)
		return;

	if(globalMDyn.pastEncoderTimeStamp == 0){
		if(angle_timestamp != 0){
			globalMDyn.pastEncoderTimeStamp = angle_timestamp;
		}
		return; //No past encoder info, do nothing
	}

	if(globalMDyn.pastEncoderTimeStamp == angle_timestamp)
		return; //last encoder reading is the same as the current one, do nothing

    // Uncomment the below statements for debug print statements to check current and desired heading

    /*
    printf("CH: %li\r\n", current_heading);
    printf("DH: %li\r\n", desired_heading);
	*/

    // Unsigned casting arithmetic handles if timer overflows - Divide by 1000 to convert from ms to s
    float del_time = (angle_timestamp - globalMDyn.pastEncoderTimeStamp) / 1000.0f;

    // Handle event if del_time = 0 - handle zero division error
    if (del_time < 0.001f) {
        del_time = 0.001f; // Minimum integration time step
    }

    // Ensure that you cannot request an angle greater than the intended design
    if (fabsf(desired_heading) > MAX_ANGLE) {
        desired_heading = desired_heading < 0 ? -MAX_ANGLE : MAX_ANGLE;
    }

    // Calculate error
    float error = desired_heading - current_heading;

    // Calculate angular velocity in deg/sec (or units/sec)
    float angular_velocity = (current_heading - globalMDyn.pastEncoderValue) / del_time;

    // Calculate current motor direction based on desired movement
    int8_t new_motor_direction = (error > 0) - (error < 0);

    // Check if error is within the acceptable threshold
	if (fabsf(error) < ERROR_THRESHOLD) {
		Set_Motor_Raw(0); // Stop the motor
		globalMDyn.integralError = 0.0f; // Reset the integral term
		globalMDyn.pastEncoderTimeStamp = angle_timestamp;  // Update timestamp
		globalMDyn.pastEncoderValue = current_heading; // Store latest encoder reading
		return;
	}

	// If angular velocity and desired direction are opposite, stop motor - required for motor driver
	if ((angular_velocity > 0 && new_motor_direction < 0) || (angular_velocity < 0 && new_motor_direction > 0)) {
		Set_Motor_Raw(0); // Stop the motor
		globalMDyn.integralError = 0.0f; // Reset the integral term
		globalMDyn.pastEncoderTimeStamp = angle_timestamp;  // Update timestamp
		globalMDyn.pastEncoderValue = current_heading; // Store latest encoder reading
		return;
	}

	if((new_motor_direction < 0 && current_heading < -HARD_ANGLE_LIMIT) || (new_motor_direction > 0 && current_heading > HARD_ANGLE_LIMIT)){
		Set_Motor_Raw(0); // Stop the motor
				globalMDyn.integralError = 0.0f; // Reset the integral term
				globalMDyn.pastEncoderTimeStamp = angle_timestamp;  // Update timestamp
				globalMDyn.pastEncoderValue = current_heading; // Store latest encoder reading
				return;
	}

    // Update the integral term
	globalMDyn.integralError += (float)error * del_time;
	globalMDyn.integralError = fminf(fmaxf(globalMDyn.integralError, -INTEGRAL_LIMIT), INTEGRAL_LIMIT); // Clamp the integral term

    // Compute the PI controller output
    float motor_output = (PROPORTIONAL_GAIN * error) + (INTEGRAL_GAIN * (globalMDyn.integralError));


    // Sets the motor to minimum power for movement if in dead zone - temporary value for the no-load motor case
    // Use DAC_Step function to determine MIN_MOTOR
//    if (fabsf(motor_output) < MIN_MOTOR) {
//        motor_output = new_motor_direction * MIN_MOTOR;
//    }
//
//    motor_output = fminf(fmaxf(motor_output, -MAX_MOTOR), MAX_MOTOR); // Clamp the motor output

    Set_Motor_Calibrated(motor_output); // Actuate the motor

    globalMDyn.pastEncoderTimeStamp = angle_timestamp; // Update the timestamp for the next iteration
    globalMDyn.pastEncoderValue = current_heading; // Store current encoder value to compute angular velocity next loop

	// Uncomment the below statements for debug print statements - place where required
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

    return;
}
