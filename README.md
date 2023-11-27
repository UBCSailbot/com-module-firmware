# STM32F407 Demo / Prototype
This is a brief STM32F407 dev board demonstration of I2C using a [VEML3328](https://www.vishay.com/docs/84968/veml3328.pdf) optical sensor and an LED. The demo video below illustrates LED dimming in response to changes in ambient light. Intead of having a set threshold, the VEML3328 sensor computes ambient light at the start of every power cycle, ensuring that the ambient light level aligns with the environmental condition that is current at.

https://github.com/UBCSailbot/com-module-firmware/assets/63937643/d8c5c91c-183c-42cf-9b6a-2ea88e23f80a

# Directory
- **/Core/Src**: Contains the source code files.
  - `/main.c`: Read the ambient light level upon initialization and dim the LED if the sensor sees less light (by changing the PWM signal).
  - `/veml3328_hardware.h`: Function prototype, variable declaration, and define command registers 
  - `/veml3328_hardware.c`: HAL I2C read/write, functions for - sensor initialization, sensor configuration, read R G B
  - `/veml3328_software.h`: Function prototype
  - `/veml3328_software.c`: Function for converting R G B to lux. As per pg 4 of [sensor application documentation](https://www.vishay.com/docs/80010/designingveml3328.pdf), we can configure the sensitivity by adjusting the gain and integration time. Also, as suggested on page 5, the G channel can be used as an ambient light sensor due to the spectral characteristics of the channel being similar to that of a human eye. Using the equation ALS (Ambient Light Sensor) lux = G x Sensitivity, we can get an estimated ambient light level. The sensitivity for this demo is set at 0.384 which comes from GAINx1/2, IT @ 50ms and DGx2. 

# How It Works
## VEML 3328
Note that this demo uses a [break out board](https://www.mikroe.com/color-10-click).

## STM32f407
### Configuration
CubeMX is a software tool that allows low-level configuration ([pins](https://microcontrollerslab.com/wp-content/uploads/2019/12/stm32f4-discovery-pinout.png) and clock).

For the purposes of this demo, the following pins were used:
- PE9: PWM
- PB8: SCL
- PB9: SDA

And the 

## Firmware
The code is built around I2C read/write and PWM.

### I2C
I2C is a serial communication protocol that uses 2 wires to transmit data, SDA (Serial Data) & SCL (Serial Clock). The SDA line, as the name suggests, transmits data bits, and the SCL line transmits the clock signal. Because it has a common clock signal shared between the [master and slave](https://www.circuitbasics.com/wp-content/uploads/2016/01/Introduction-to-I2C-Single-Master-Single-Slave.png), the protocol is synchronous (does real-time communication). 

The [message structure](https://www.circuitbasics.com/wp-content/uploads/2016/01/Introduction-to-I2C-Message-Frame-and-Bit-2.png) of I2C has:
- **Start Condition:** Occurs when the SDA line transitions from a high (typically 3.3V or 5V) to low voltage before the SCL line transitions from high to low.

- **Address Frame:** A 7 to 10-bit sequence used to identify slave devices. Addressing is how I2C handless multiple device communication without a separate line like SPI.

- **Read/Write:** A single bit signalling the slave whether the master intends to write data into it or receive data from it.

- **Data Frame:** An 8-bit long frame containing the data that is to be transmitted.

- **ACK/NACK:** Each message frame is succeeded by an acknowledgment bit from the receiving device, indicating the successful reception of the frame.

- **Stop Condition:** Occurs when the SDA line transitions from low to high voltage after the SCL line transitions from low to high.

### Pulse Width Modulation
Pulse Width Modulation (PWM) is a technique for generating continuous HIGH/LOW (ON/OFF) signals. By controlling both the width and frequency of the pulses, the power delivered to a load can be easily adjusted. This method is commonly used to control the speed of motors or regulate the brightness of LEDs.

- Frequency: Measured in Hertz (Hz), frequency represents the speed of alternation between the HIGH and LOW states in a signal. A higher frequency indicates a faster alternation. 
https://cdn-0.deepbluembedded.com/wp-content/uploads/2023/05/Arduino-PWM-Signal-DutyCycle-Frequency-Equation.jpg?ezimgfmt=ng:webp/ngcb6

- Duty Cycle: Measured in percentage (%), duty cycle is the duration of the HIGH (ON) state relative to the signal's period. For example, a 15% duty cycle implies that the PWM signal's voltage is high for 15% of the signal period and low for the remaining 85%.
https://cdn-0.deepbluembedded.com/wp-content/uploads/2023/05/Arduino-PWM-Duty-Cycle-Equation.jpg?ezimgfmt=ng:webp/ngcb6

The resolution of the signal is a crucial factor, defining the maximum number of steps in one PWM period. Higher resolution allows for finer adjustments in duty cycles. In practical terms, a resolution of 3 bits means 8 discrete levels (2^3), while 8 bits mean 256 discrete levels. 
https://www.eeweb.com/wp-content/uploads/articles-articles-pwm-1374007966.png 

It's important to consider the relationship between frequency and resolution. In some cases, higher PWM resolution might require more frequent updates of the PWM signal to achieve finer adjustments in the duty cycle. If you try to increase the resolution without adjusting the frequency, you might end up with a PWM signal that changes too slowly to represent the desired duty cycle accurately. Conversely, if a higher PWM frequency is required, one might have to compromise resolution to maintain real-time performance due to hardware limitations. This trade-off often arises from constraints in processing power or the capabilities of PWM modules. Here is an example that studies various frequency and resolution options for [PIC17C42](http://www.t-es-t.hu/download/microchip/an539c.pdf)

#### CUBE MX Configuration

### Hardware Abstraction Layer (HAL) Driver
```C
HAL_StatusTypeDef veml3328_wr(uint8_t registerAddress, uint16_t value)

HAL_StatusTypeDef veml3328_rd(uint8_t registerAddress, uint16_t* value)

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
```

# Resources
- [STM32 PWM](https://deepbluembedded.com/stm32-pwm-example-timer-pwm-mode-tutorial/)
