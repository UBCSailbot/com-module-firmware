# Communication Module Firmware (COM Module)

The Communication Modules are microcontrollers with the specific purpose of providing bidirectional communication between a device/sensor on the boat and the main computer. It processes these messages and provides low-level interfacing and control of individual hardware devices on the boat. Examples such applications include rudder and wingsail controls, and data aquisition from various sensors on the boat.

This repository consists of multiple firmware projects from the electrical team working on the project Polaris at UBC Sailbot. All of the firmware is written for the [STM32U5 Nucleo-144](https://www.st.com/en/evaluation-tools/nucleo-u575zi-q.html) boards.

Current projects include:

1. Base communication module library
2. Rudder controller firmware
3. Wingsail controller firmware

![High-level diagram of communication system](shared_docs/images/HL_COM_Diagram.png)

## COM Module High-level Design

The COM Module is split into two parts: The Nucleo board and the breakout board (BOB). Each hardware device on the boat has it's own custom BOB designed specifically for it's needs, while the Nucleo board hardware is the same in all COM Modules.

<!-- ![COM module high-level](shared_docs/images/COM_Internals_Diagram.png) -->

![alt text](shared_docs/images/COM_Internals_Diagram2.png)

## Firmware Design Diagram
<img width="693" alt="image" src="https://github.com/UBCSailbot/com-module-firmware/assets/144284916/f6985165-35a1-43e2-b885-3d951ad07747">

## Repository Structure

<!--
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
-->

```
projects/
    base-library/
        project/
            Core/
                Inc                        - Contains base library header files
                Src                        - Contains base library source files
        tests
        README.md                          - Base library user instructions
    rudder-controller
    template                               - Template to be copied for teams writing their own code
shared-docs
tutorials/
    STM32F407_demo/                        - Comprehensive demo project
        README.md                          - Start here for the demo
    setup.md                               - Tutorial for STM32CubeIDE setup
    testing.md                             - Tutorial for how to create tests
.gitignore
LICENSE
README.md                                  - Relevant background info on COM Modules (you're here right now)
```

## Where to Get Started
If you are looking to write your own firmware for your team, then you have come to the right place. After looking over this page, navigate to ```projects -> base-library -> README.md``` for another README, this time with specific instructions on what you need to know. As you can see above, there are multiple other files that are worth looking at. 

## Prerequisites

List of tools required to get started:

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - Used to generate initial code and peripherial configurations using the in-built STM32CubeMX.
- [Visual Studio Code](https://code.visualstudio.com/) - Alternative IDE for editing code (rather than CubeIDE).
- [Putty](https://putty.org/) - Used for serial communication between host and STM32U5 board.
- [Git](https://git-scm.com/downloads) - For version control.

If you have not setup your working environment yet, follow the [setup instructions](tutorials/setup.md).
For information on working with GitHub, consult the [github instructions](tutorials/github.md).

<!-- ## Resources -->
