# Communication Module Firmware (COM Module)

The Communication Modules are microcontrollers with the specific purpose of providing bidirectional communication between a device/sensor on the boat and the main computer. It processes these messages and provides low-level interfacing and control of individual hardware devices on the boat. Examples such applications include rudder and wingsail controls, and data aquisition from various sensors on the boat.

This repository consists of multiple firmware projects from the electrical team working on the project Polaris at UBC Sailbot. All of the firmware is written for the [STM32U5 Nucleo-144](https://www.st.com/en/evaluation-tools/nucleo-u575zi-q.html) boards.


## COM Module High-level Design

The COM Module is split into two parts: The Nucleo board and the breakout board (BOB). Each hardware device on the boat has it's own custom BOB designed specifically for it's needs, while the Nucleo board hardware is the same in all COM Modules.

## Repository Overview
The repositroy contains two main components, modules and projects. 
### Projects
Projects are entire STM32 projects for each physical controller on the boat. There should not be more of these then there are NUCLEO boards on the boat.
### Modules
Modules are individual pieces of code for controlling a specific aspect. Examples: CANBUS, a windsensor, motor control, etc. The majority of the code should be in modules, the project only serves to connect modules together and deal with high level tasks. The module should include a README file that at minimum explains the purpose of the code, the intended usage and how to configure the IOC file. For example if you are making a module to read from an encoder, anyone with experience in STM32 and context of sailbot should be able to use the module just from reading the README file (although they will need to figure out things like wiring or circuitry on their own).
### Structure
```
root/
    README.md
    .gitignore
	.gitattributes
    LICENSE
    resources/                           	- Some useful datasheets / pdfs for easy access
    code/
        controller-projects/				- STM32 projects for each physical controller on the boat
			wingsail-controller/
			sense-controller/
			...
		drv-modules/ 						- Modules made by DRV for DRV related tasks
			nmea0183/
				nmea0183.h					- Module header file
				nmea0183.c					- Module source file
				README.md					- Module README. At minimum should explain purpose, usage and how to configure (especially the IOC file)
			briter-encoders/
			...
		com-modules/ 						- Modules made by COM for COM related tasks				
			can-library/
			...
		pwr-modules/ 						- Modules made by PWR for PWR related tasks
			current-sensor/
			...
			
```

## Linking modules to controller-projects

```controller-projects``` is where whole projects are created, while ```___-modules``` is where small modules are written.

To flash a controller project with a module, you must first link the projects together:

1. Open the desired project (for example rudder-controller)
2. Right click on the project and go to ```properties```
3. Inside properties, go to ```C/C++ General -> Paths and Symbols```
4. First, go to ```Source Location```
5. Hit ```Link Folder```
6. Under ```Advanced``` choose ```Link to folder in the file system```
7. Enter a **relative** path to the module (for example: ```../../drv-modules/nmea0183```)
8. Navigate to ```Includes``` in ```Paths and Symbols```
9. Choose ```Add...```
10. Add another **relative** path to the same module (for example: ```../../drv-modules/nmea0183```)

The module should now appear your controller project!

### IF ALL YOUR MODULES DISSAPEAR

Don't fear. ```Run as``` will call ```make -j8``` which will make all the modules show up again.


## Prerequisites

List of tools required to get started:

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Used to generate initial code and peripherial configurations using the in-built STM32CubeMX.
- [Visual Studio Code](https://code.visualstudio.com/) - Alternative IDE for editing code (rather than CubeIDE).
- [Putty](https://putty.org/) - Used for serial communication between host and STM32U5 board.
- [Git](https://git-scm.com/downloads) - For version control.
