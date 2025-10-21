# Overview
This document will outline how to use the BRITER library and set up the required peripherals

# IOC Setup
Unless otherwise noted all IOC settings are that of default. Note that different UART peripherals or DMA channels may be used although they are untested and should be tested before use.
## UART
* Using the `USART2` peripheral
* Set the baud rate to `9600 Bits/s`
* Under Advanced Features -> TX Pin Active Level Inversion should be `True`
* Under Advanced Features -> RX Pin Active Level Inversion should be `True`
* In NVIC Settings the `USART2 Global Interrupts` should be enabled
## GPDMA1
* Channel 9 should be set to `Standard Request Mode`
* Under Request Configuration -> Request should be set to `USART2_RX`
* Under Destination Data Setting -> Destination Address Increment After Transfer should be `Enabled`

# Code Example
## Setup
* In the user includes add the header file:
```
#include "BRITER.h"
```
* Declare a global pointer for the BRITER datatype. I did this in user code 0:
```
BRITER * encoderObject;
```
* After the USART peripherals have been set up but before the infinite loop add the following code. This will declare the angle register and set up the BRITER object. I put this in user code 2:
```
volatile uint16_t encoderAngle = 0;
encoderObject = BRITER__create(&huart2, &encoderAngle, 50);
```
## Callback
Add the following code after your main loop. Typically this is put in user code 4:
```
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	BRITER__handleDMA(encoderObject, huart, size);
}
```
Other code can be put in these functions for other peripherals/timers and their required callbacks, but these functions must be called. Without these, the data collection will not happen.
## Getting Data
The `encoderAngle` variable will always have the up-to-date heading information since it was sent to the BRITER__create function. Note this variable may be modified at any time.
## Zeroing the Encoder
Simply use the function as follows. Note this function works in blocking mode and should not be used except to initially zero the encoder.
```
BRITER__zeroPosition(encoderObject);
```
