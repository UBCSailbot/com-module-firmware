# Wingsail Firmware
This is the beginning of the wingsail firmware. The readme will (hopefully) be expanded when more things are added.
# Setup
## IOC Connectivity USART2
* USART2 -> Mode -> Asychronous
* USART2 -> Configuration -> Parameter Settings -> Basic Parameters -> Baud Rate = 4800 or 38400 are standard. Have seen devices in the past with 19200 as well.
* USART2 -> Configuration -> NVIC Settings -> USART2 Global Interrupts = Enabled
## IOC GPDMA1
* Mode -> Channel 15 = Standard Request Mode
* Configuration -> All Channels -> Channel 15 -> Request = GPDMA1_REQUEST_USART2_RX
* Configuration -> CH15 -> Request Configuration -> Request = USART2_RX
* Configuration -> CH15 -> Channel Configuration -> Direction = Peripheral to Memory
* Configuration -> CH15 -> Source Data Setting -> Source Address Increment After Transmit = Disabled
* Configuration -> CH15 -> Destination Data Setting -> Destination Address Increment After Transmit = Enabled
## main.h
Add the include in here so it has scope inside of `stm32u5xx_it.c`
## stm32u5xx_it.c
In `USER CODE BEGIN USART2_IRQn 0` add `NMEA0183__IRQHandler(&huart2);`
