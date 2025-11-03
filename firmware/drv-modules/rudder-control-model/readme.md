# Overview
[comment]: #(Feel free to add things to this readme if they do not fit well in either the .c or .h file.
Some examples of what might be good to include:
* Setup if things need to be initialized (ex: DMA channels, Timers, Interrupts, etc.)
* Example of how to implement things
* etc.)

# Structure of the Control Model
The control model is essentially a very simple state machine that determines the boat's sailing state and cycles through accordingly.

There is one key function, runPID, that is intended to be called in the main loop once per cycle. This is the function that contains the state machine and will execute everything accordingly.

There are currently four states implemented - straight line, tacking, gybing, and low winds. Irons has yet to be implemented. Each of these states has it's own coefficients that must be initialised and tuned separately.

The control model functionality is contained in RUDDER.c, while the external fixed parameters are to be stored in RUDDER_PARAMS.c. RUDDER_PARAMS.h contains functions to get, set, and edit PID paramters and the like.

# Variables and Such

The control model contains an PIDController struct "controller" that consists of a live and a fixed struct. The live struct has all of the changing info for the controller - PID integral and time info, wind, heading, etc. The fixed struct is the parameteres that we must tune/ set - physical limits, PID coefficients, scaling factors, etc. 

The functions within RUDDER.c work by updating and retrieving info from this struct. This struct can be initialised through the initController function. You must pass in a PIDControllerFixed struct to this function - this should be the struct from RUDDER_PARAMS.

Once this has been initialized, pretty well the only function needed should be run_PID and updateControllerVariables, which should each be called once per loop. The resetController function can be used to reset the controller (as it says on the tin).

## Sensors
Currently there are just placeholder functions for sensor values. These need to be implemented in the near future/ as things get finalised.

We are considering how various updates at different times should be handled. This is not currently being handled.


