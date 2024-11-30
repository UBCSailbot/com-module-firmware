
/*
 * This library is designed to set 
 * 
 *
 *
 *  Created on: November 30, 2024
 *      Author: Chukwudalu Joshua Obi
 */

#define PROPORTIONAL_GAIN 0.0001
#define INTEGRAL_GAIN 0.0001
#define ERROR_THRESHOLD 1
#define INTEGRAL_LIMIT 1

// Need to create the follwoing pointers to run the following functions 
// float *integral_error;
// float *last_time_stamp;

/*
 * Sets the direction and speed of the motor controller using the STM32 -U575
 *
 * @param Motor_Control a float value between -1.0 and 1.0
 * Positive to indicate one direction and negative in the other direction - sets output on pin C8
 * Magnitude of float value indicates speed - sets output on DAC1_Channel 2 - sets output on pin A5
 */

void Set_Motor(float_t Motor_Control);

/*
 * Calculates the actuation for the motor depending on what the desired heading is using the STM32 -U575
 *
 * @param desired heading - an angle provided by Rudder Controller PID
 * @param current heading - an angle provided by the encoder 
 * Calculates the actuation of the motor to meet the desired heading
 */

void PI_Motor(int32_t desired_heading, int32_t current_heading);

void Set_Motor(float_t Motor_Control)
{
    // Sets DAC value between 0 and 4095 depending on magnitude of Motor_Control
    // to change speed
    uint32_t Motor_DAC = (uint32_t)(fabsf(Motor_Control) * 4095.0f); // Use uint32_t and cast explicitly
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Motor_DAC);

    // Sets PC8 high if Motor_Control is positive, otherwise low
    if (Motor_Control > 0.0f)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    }
}

void PI_Motor(int32_t desired_heading, int32_t current_heading)
{
    int32_t error = desired_heading - current_heading; // Use int32_t for error
    int32_t direction;

    // Determine the direction based on error sign
    if (error < 0)
    {
        direction = -1;
    }
    else
    {
        direction = 1;
    }

    // Check if error is within the threshold range
    if (fabsf((float)error) < ERROR_THRESHOLD) // Cast error to float for comparison
    {
        Set_Motor(0.0f); // Stop the motor
        *integral_error = 0.0f; // Reset the integral term
        *last_time_stamp = HAL_GetTick(); // Update timestamp
        return;
    }
    else
    {
        // Find the current time
        uint32_t current_time_stamp = HAL_GetTick();

        // Calculate the time difference
        uint32_t del_time = (uint32_t)(current_time_stamp - *last_time_stamp);

        // Add the error to the integral sum
        *integral_error += ((float)error * (float)del_time);

        // Clamp the integral sum to prevent windup
        if (*integral_error > INTEGRAL_LIMIT)
        {
            *integral_error = INTEGRAL_LIMIT;
        }
        else if (*integral_error < -INTEGRAL_LIMIT)
        {
            *integral_error = -INTEGRAL_LIMIT;
        }

        // Calculate the motor output
        float_t motor_output = direction * (PROPORTIONAL_GAIN * (float)error + INTEGRAL_GAIN * (*integral_error));

        // Set the motor actuator
        Set_Motor(motor_output);

        // Update the timestamp for the next error calculation
        *last_time_stamp = current_time_stamp;

        return;
    }
}