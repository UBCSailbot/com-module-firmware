# Overview
Feel free to add things to this readme if they do not fit well in either the .c or .h file.
Some examples of what might be good to include:
* Setup if things need to be initialized (ex: DMA channels, Timers, Interrupts, etc.)
* Example of how to implement things
* etc.


# Structure of the Control Model
Probably delete this section in the readme once you have implemented things. These are my thoughts only on the high level structure, feel free to ignore this as you see fit. One thing that is critical is that nothing in here should take a "long" time to execute. What I mean is no delays in the code or infinite loops. Otherwise there might be sensor data or something that we miss.
## Initialization
Some function for initializing things as needed
## Cycle through the PID
Probably have a function that runs one iteration on the PID model. 
* We'll probably set this up so that it get's called on a defined period (e.i every 5ms)
* I would recommend updating the integral and derivative terms assuming that it is not called on this regular period (in case an interrupt or something delays execution), use some kind of a timer
* This function would then call the motor to move as needed (feel free to look at Joshua's code on how to call his functions, or just put in a placeholder function)
## Input Data
Have functions for each sensor to input data.
* These functions would be called whenever there is new input data, and not nessacrily on a regular period
* You would want a function for: IMU data, Wind sensor 1 data, Wind sensor 2 data, linear velocity, desired heading, and possibly others I am forgetting
* Feel free to look at the `CAN Frames` confluence page for what the input to these functions might look like (it can easily be changed though)



