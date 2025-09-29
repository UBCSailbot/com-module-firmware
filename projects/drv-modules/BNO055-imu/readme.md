# Overview
This document will outline how to use the IMU library and set up the required peripherals.

# IOC Setup
Unless otherwise noted all IOC settings are that of default. Note that different I2C peripherals, different DMA channels or different timers may be used although they are untested and should be tested before use.
## I2C
* Using the `I2C2` peripheral
* Set the I2C mode to `I2C`
* In NVIC Settings the `I2C2 Event interrupt` should be enabled
## GPDMA1
### Channel 10
* Mode should be set to `Standard Request Mode`
* Under Request Configuration -> Request should be set to `I2C2_TX`
### Channel 11
* Mode should be set to `Standard Request Mode`
* Under Request Configuration -> Request should be set to `I2C2_RX`
* Under Destination Data Setting -> Destination Address Increment After Transfer should be set to `Enabled`
## Timer
* Using the `TIM2` timer
* Set the Clock Source to `Internal Clock`
* In NVIC Settings the `TIM2 global interrupt` should be enabled

# Code Example
## Setup
* In the user includes add the header file:
```
#include "IMU.h"
```
* Declare a global pointer for the IMU datatype. I did this in user code 0:
```
IMU * imu;
```
* After the I2C peripheral and timer have been set up and before the infinite loop, add the following code. This will declare your heading register, setup your IMU object and start the timer that controls the data collection. I put this in user code 2:
```
volatile uint32_t headingData = 0;
imu = IMU__create(&hi2c2, &htim2 ,5, &headingData);
HAL_TIM_Base_Start_IT(&htim2);
```
## Callbacks
Add the following code after your main loop. Typically this is put in user code 4:
```
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	IMU__handleTxDMA(imu, I2cHandle);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle){
	IMU__handleRxDMA(imu, I2cHandle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	IMU__updateBuffer(imu, htim);
}
```
Other code can be put in these functions for other peripherals/timers and their required callbacks, but these functions must be called. Without these the data collection will not happen.

## Getting Data
The `headingData` variable will always have the up-to-date heading information since it was sent to the IMU__create function. Note this variable may be modified at any time.
