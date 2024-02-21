# Communication Module Firmware (COM Module)

The Communication Module is a microcontroller for the specific purpose of providing bidirectional communication between a device/sensor on the boat with the main computer. It processes these messages and provides low-level interfacing and control of individual hardware devices on the boat. Examples such applications include rudder and wingsail controls, and data aquisition from various sensors on the boat.

This repository consists of multiple firmware projects from the electrical team working on the project Polaris at UBC Sailbot. All of the firmware is written for the [STM32U5 Nucleo-144](https://www.st.com/en/evaluation-tools/nucleo-u575zi-q.html) boards.

Current projects include:

1. Base communication module library
2. Rudder controller firmware
3. Wingsail controller firmware

![High-level diagram of communication system](shared_docs\HL_COM_Diagram.png)

## COM Module High-level Design

The COM Module is split into two part. The Nucleo board and the BOB (breakout board). Each hardware device on the boat has it's own custom BOB designed suited for it's needs. The Nucleo board hardware is the same in all COM Modules

![COM module high-level](shared_docs\COM_Internals_Diagram.png)

![alt text](shared_docs\COM_Internals_Diagram2.png)

## Repository Structure

```
root/
    README.md
    .gitignore
    LICENSE
    tutorial/                              - Setup instructions and tutorials
    shared docs/                           - Common/shared documents between projects
    setup_instructions.md
    scripts/                               - Automated testing and deployment scripts
    projects/
        project1/
            project/                          - Main CubeIDE Project
            tests/                            - Component level unit tests
                COMPONENT1/
                COMPONENT2/
            docs/                             - Project technical documentation
                architecture.md
                testing_instructions.md
                datasheets/
                    component1.pdf
        project2/
```

## Prerequisites

List of tools required to get started:

- [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Used to generate initial code and peripherial configurations using the in-built STM32CubeMX.
- [Visual Studio Code](https://code.visualstudio.com/) - Alternative IDE for editting code option to Cube IDE.
- [Putty](https://putty.org/) - Used for serial communication between host and STM32U5 board.
- [Git](https://git-scm.com/downloads) - Version control.

## Resources
