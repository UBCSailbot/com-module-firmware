# Communication Module Firmware (COM Module)

The Communication Modules are microcontrollers with the specific purpose of providing bidirectional communication between a device/sensor on the boat and the main computer. It processes these messages and provides low-level interfacing and control of individual hardware devices on the boat. Examples such applications include rudder and wingsail controls, and data acquisition from various sensors on the boat.

This repository consists of multiple firmware projects from the electrical team working on the project Polaris at UBC Sailbot. All of the firmware is written for the [STM32U5 Nucleo-144](https://www.st.com/en/evaluation-tools/nucleo-u575zi-q.html) boards.


## COM Module High-level Design

The COM Module is split into two parts: The Nucleo board and the breakout board (BOB). Each hardware device on the boat has its own custom BOB designed specifically for its needs, while the Nucleo board hardware is the same in all COM Modules.

# Repository Overview

We do not require an in-depth understanding of version control and git, but you do need to know the basics. There are plenty of sources online for how to use git; feel free to browse around. At the very least, you should have an understanding of:
* Branches
* Commits
* Pull Requests / Merging
* Merge Conflicts
* The following commands ```pull```, ```fetch```, ```checkout```, ```push```, ```commit```, ```add```, ```restore```, ```branch```, ```init```, ```status```, ```clone``` and ```reset```

## Version Control Methodology

**Feature Branch:** For each high-level feature (AIS code, windsensor code, CANBUS code, etc.), while it is in development, it should have its own branch. The idea is that each person works in one or more of their own feature branche(s). This allows one to make code without having to worry about conflicts of working in the same branch. When code needs to be integrated, it can be merged into a working branch. Mainly, people should be doing the majority of their work in a feature branch, and when code needs to be merged to a working branch, it should be communicated to anyone else working on a feature for that working branch. The exception to this is if you know you are the only one working on a particular physical COM module, you may work directly in the working branch. This branch should be named something indicative of what it does. (Examples: AIS-firmware, CANBUS-library, PID-controller, ...)

**Working Branch:** For each physical COM module, there should be a working branch where code is being assembled and integrated. This can be rough code, but it should at least compile and be useful code. These branches should be named as the COM module description, followed by "working-branch". (Examples: windsail-working-branch, rudder-working-branch, sense-working-branch, PDB-working-branch)

**Main Branch:** This is where our finalized, reviewed code goes. Only properly documented, clean, and functional code should be included in (the caveat being that some pre-existing code may not follow this standard). To get code in the main branch, you must make a PR with two people signing off on it, one of whom should be an ELEC lead.

## File Structure
The repository contains two main components, modules and projects:

### firmware

All firmware goes in the firmware folder.

**Projects** are entire STM32 projects for each physical controller on the boat. There should not be more of these than there are NUCLEO boards on the boat.

**Modules** are individual pieces of code for controlling a specific aspect. Examples: CANBUS, a windsensor, motor control, etc. The majority of the code should be in modules; the project only serves to connect modules and deal with high-level tasks. The module should include a README file that, at a minimum, explains the purpose of the code, the intended usage, and how to configure the IOC file. For example, if you are making a module to read from an encoder, anyone with experience in STM32 and the context of sailbot should be able to use the module just from reading the README file (although they will need to figure out things like wiring or circuitry on their own).

### software-tools

This is where all external diagnostics/software tools go. For example, we have a remote controller for the whole boat that runs on a diagnostic laptop over wifi. We also have bootloading scripts for uploading from the PI to NUCLEOs, and in the past have used simple Python scripts for testing. If a software tool is specific to a COM module, when it comes to version control, it should be treated the same as a feature branch. If it is boat-wide, then it should be treated the same way as a working branch.

### resources

Put any firmware resources here if they are small in file size; otherwise, they should go in Google Drive.

### Structure

```
root/
    README.md
    .gitignore
	.gitattributes
    LICENSE
    resources/                           	- Some useful datasheets / PDFs for easy access
		...
    firmware/
        controller-projects/				- STM32 projects for each physical controller on the boat
			wingsail-controller/
			sense-controller/
			...
		drv-modules/ 						- Modules made by DRV for DRV-related tasks
			nmea0183/
				nmea0183.h					- Module header file
				nmea0183.c					- Module source file
				README.md					- Module README. At a minimum should explain the purpose, usage, and how to configure (especially the IOC file)
			briter-encoders/
				...
			...
		com-modules/ 						- Modules made by COM for COM-related tasks				
			can-library/					- Example
				...
			...
		pwr-modules/ 						- Modules made by PWR for PWR-related tasks
			current-sensor/					- Example
				...
			...
	software-tools/							- All non-firmware diagnostic or control utilities.
		remote-boat-controller/
			...
		bootloader/
			...
			
```

## Linking modules to controller-projects

```controller-projects``` is where whole projects are created, while ```___-modules``` is where small modules are written.

To flash a controller project with a module, you must first link the projects together:

1. Open the desired project (for example, rudder-controller)
2. Right-click on the project and go to ```properties```
3. Inside properties, go to ```C/C++ General -> Paths and Symbols```
4. First, go to ```Source Location```
5. Hit ```Link Folder```
6. Under ```Advanced``` choose ```Link to folder in the file system```
7. Enter a **relative** path to the module (for example: ```../../drv-modules/nmea0183```)
8. Navigate to ```Includes``` in ```Paths and Symbols```
9. Choose ```Add...```
10. Add another **relative** path to the same module (for example: ```../../drv-modules/nmea0183```)

The module should now appear in your controller project!

### IF ALL YOUR MODULES DISSAPEAR

Don't fear. ```Run as``` will call ```make -j8``` which will make all the modules show up again.


## Prerequisites

List of tools required to get started:

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Used to generate initial code and peripheral configurations using the in-built STM32CubeMX.
- [Visual Studio Code](https://code.visualstudio.com/) - Alternative IDE for editing code (rather than CubeIDE).
- [Putty](https://putty.org/) - Used for serial communication between host and STM32U5 board.
- [Git](https://git-scm.com/downloads) - For version control.
