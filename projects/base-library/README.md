# Firmware Base Library

This page is for the communication modules base library, and includes explanations and instructions for its users. Don't forget to also read the main README.

## High-level Design

<img width="692" alt="image" src="https://github.com/UBCSailbot/com-module-firmware/assets/144284916/c1f9b2fc-40b4-414a-8d1a-b80418eed409">

### What is the Base Library?
The base library is made up of files corresponding to the boxes in the image above. Namely, they are CAN, Board, Debug, Error, and Config. The purpose of this code is to provide the "outline" for teams to build off of. It contains functions that facilitate communication through CAN, I2C, ADC, PWM, and UART, as well as provides debug and error handling capabilities. Lastly, the configuration file provides unique customizability, and avoids including unneeded code in other teams designs, thereby optimizing performance.

Here is what you need to know about which features/functions you can find, and where:

```can.h``` - provides functionality to create CAN message frames to be sent, as well as parse and extract data from received frames (the way it does this depends on the device being communicated with, which will be specified in the config file) 

```board.h``` - provides basic functions for CAN, I2C, ADC, PWM, and UART communication

```debug.h``` - provides debugging functions such as printing/storing logs

```error.h``` - provides functions that properly handle errors in operations such as CAN communication or issues with receiving sensor data

```config.h``` - specifies CAN/I2C frames, PWM rates, and defines what is being used, so that the rest can be excluded - NOTE: this file is to be edited by the user

As you probably notice, the location of each feature is quite intuitive, especially once you've seen the diagram above.

To find these files, navigate through the repository as follows: ```projects -> base-library -> project -> Core -> Inc -> xxxxx.h```

## Functions

```board.c```

<br/>

```write()``` - function needed to print with UART

<br/>

```pwm1_init_ch1()``` - sets the duty cycle of PWM1

```pwm1_init_ch1()``` - sets the duty cycle of PWM3

```pwm1_set_ch1()``` - sets the duty of PWM1

```pwm3_set_ch1()``` - sets the duty of PWM3

<br/>

```gpio_rd_e2()``` - reads pin 2

```gpio_rd_e4()``` - reads pin 4

```gpio_wr_e3()``` - writes to pin 3

```gpio_rs_e3()``` - resets pin 3

```gpio_wr_e5()``` - writes to pin 5 

```gpio_rs_e5()``` - resets pin 5



## User Manual
Before continuing, please read the "What is the Base Library" paragraph above if you haven't already.

The three boxes at the top of the firmware design diagram illustrate the tasks of the individual teams. To reach the desired functionality for a specific COM module, such as reading data from a sensor, teams have to determine which functions are required, and what the proper inputs to those functions would be. Most important for this would be the ```board.h``` file, but the comprehensive list above covers everything available.

There are also features that you can request from the firmware team. If there is any specific functionality you would like for handling errors, such as turning off a sensor if we can no longer receive data from it, then leave your request on the [Confluence feature request page](https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1994457093/Feature+Request). Additionally, any other requests for code that needs new functions/functionality that isn't included in the base library can also be left in the same place. 

Lastly, you can change values in the configurations file based on what you need and don't need from the base library. This is an important consideration as it will improve the performance of your COM Module!

## Notes
You can find tutorials on using STM32CubeIDE, GitHub, and more in the tutorials folder.

Don't forget to consult the main README to learn more.

