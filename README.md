# STM32F407 Demo / Prototype
This is a quick demonstration of I2C on the STM32F407 dev board. It uses a veml3328 optical sensor and an LED with an on-board PWM generator to adjust the light in correspondence to the change in light level.

https://github.com/UBCSailbot/com-module-firmware/assets/63937643/d8c5c91c-183c-42cf-9b6a-2ea88e23f80a

# Directory
- **/Core/Src**: Contains the source code files.
  - `/main.c`: Read the ambient light level upon initialization and dim the LED if the sensor sees less light (by changing the PWM signal).
  - `/veml3328_hardware.h`: Function prototype, variable declaration, and define command registers 
  - `/veml3328_hardware.c`: HAL I2C read/write, functions for - sensor initialization, sensor configuration, read R G B
  - `/veml3328_software.h`: Function prototype
  - `/veml3328_software.c`: Function for converting R G B to lux. As per pg 4 of [this](https://www.vishay.com/docs/80010/designingveml3328.pdf) sensor application documentation, we can configure the sensitivity by adjusting the gain and integration time. Also, as suggested on page 5, the G channel can be used as an ambient light sensor due to the spectral characteristics of the channel being similar to that of a human eye. Using the equation ALS (Ambient Light Sensor) lux = G x Sensitivity, we can get an estimated ambient light level. The sensitivity for this demo is set at 0.384 which comes from GAINx1/2, IT @ 50ms and DGx2. 

# How It Works
## VEML 3328
The VEML3328 is a colour sensor that detects R, G, B, C, and IR using the I2C communication protocol. The datasheet can be found [here](https://www.vishay.com/docs/84968/veml3328.pdf).

Note that this demo uses a dev board; information is available [here](https://www.mikroe.com/color-10-click).

## STM32f407
### Peripherals 
STM32 offers software known as CubeMX, which allows the configuration of pins to streamline development processes. This tool ensures that only the necessary pins/functions are activated, and it provides pre-written function templates.

#### Pinout
Refer to the board pinout diagram [here](https://microcontrollerslab.com/wp-content/uploads/2019/12/stm32f4-discovery-pinout.png) for a comprehensive overview of available pins.

#### Demo Configuration
For the purposes of this demo, the following pins should be activated:

    PE9: PWM
    PB8: SCL
    PB9: SDA

Alternatively, developers can directly use the provided .ioc file in this repo.

## Firmware
This is a simple demo. The codes are built around two main functions: I2C read/write and PWM signal generator.

### I2C
I2C is a serial communication protocol that uses 2 wires to transmit data, SDA (Serial Data) & SCL (Serial Clock). The SDA line, as the name suggests, transmits data bits, and the SCL line transmits the clock signal. And because it has a common clock signal shared between the [master and slave](https://www.circuitbasics.com/wp-content/uploads/2016/01/Introduction-to-I2C-Single-Master-Single-Slave.png), the protocol is synchronous (real-time communication). 

The [message structure](https://www.circuitbasics.com/wp-content/uploads/2016/01/Introduction-to-I2C-Message-Frame-and-Bit-2.png) of I2C is simple. There are:
- **Start Condition:** Occurs when the SDA line transitions from a high (typically 3.3V or 5V) to low voltage before the SCL line transitions from high to low.

- **Address Frame:** A 7 to 10-bit sequence used to identify slave devices. Addressing is how I2C allows multiple device communication without a separate line like SPI.

- **Read/Write:** A single bit signalling the slave whether the master intends to write data into it or receive data from it.

- **Data Frame:** An 8-bit long frame containing the data that is to be transmitted.

- **ACK/NACK:** Each message frame is succeeded by an acknowledgment bit from the receiving device, indicating the successful reception of the frame.

- **Stop Condition:** Occurs when the SDA line transitions from low to high voltage after the SCL line transitions from low to high.

### Pulse Width Modulation

### Hardware Abstraction Layer (HAL) Driver
```C
HAL_StatusTypeDef veml3328_wr(uint8_t registerAddress, uint16_t value)

HAL_StatusTypeDef veml3328_rd(uint8_t registerAddress, uint16_t* value)

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
```
