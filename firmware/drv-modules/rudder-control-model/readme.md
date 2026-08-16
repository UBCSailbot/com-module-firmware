# Overview
[comment]: #(Feel free to add things to this readme if they do not fit well in either the .c or .h file.
Some examples of what might be good to include:
* Setup if things need to be initialized (ex: DMA channels, Timers, Interrupts, etc.)
* Example of how to implement things
* etc.)

# Structure of the Control Model
The control model is split into three pieces: rudder PID output, state estimation, and sailing state transitions.

There is one key function, runPID, that is intended to be called in the main loop once per cycle. This function coordinates estimation, state-machine updates, and the preserved PID output logic.

There are currently four states implemented - straight line, tacking, gybing, and low winds. Irons has yet to be implemented. Each of these states has it's own coefficients that must be initialised and tuned separately.

The PID output and state-specific rudder commands are contained in RUDDER.c. Sailing-state transition logic is contained in STATE_MACHINE.c. Estimation and moving averages are contained in ESTIMATOR.c. The external fixed parameters are stored in RUDDER_PARAMS.c. RUDDER_PARAMS.h contains functions to get, set, and edit PID parameters and the like.

# Variables and Such

The control model contains an PIDController struct "controller" that consists of a live and a fixed struct. The live struct has all of the changing info for the controller - PID integral and time info, wind, heading, etc. The fixed struct is the parameteres that we must tune/ set - physical limits, PID coefficients, scaling factors, etc. 

The functions within RUDDER.c work by updating and retrieving info from this struct. This struct can be initialised through the initController function. You must pass in a PIDControllerFixed struct to this function - this should be the struct from RUDDER_PARAMS.

Once this has been initialized, pretty well the only function needed should be run_PID and updateControllerVariables, which should each be called once per loop. The resetController function can be used to reset the controller (as it says on the tin).

## Sensors
Currently there are just placeholder functions for sensor values. These need to be implemented in the near future/ as things get finalised.

We are considering how various updates at different times should be handled. This is not currently being handled.

## Unit Tests

Host-side unit tests for state-machine transitions and PID controller behavior live in `tests/test_rudder_control.c`.

On Windows, run:

```powershell
powershell -ExecutionPolicy Bypass -File firmware/drv-modules/rudder-control-model/tests/run_tests.ps1
```

On systems with `make`, run:

```sh
make -C firmware/drv-modules/rudder-control-model/tests test
```
