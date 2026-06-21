# PLRS-IMU rudder link

Receives the one-way heading link from the PLRS-IMU over UART and exposes the
latest heading plus its age. The wire format (COBS framing, CRC-16/CCITT-FALSE,
little-endian) is defined in the PLRS-IMU repository's `docs/rudder_link.md`.

Two layers:

- `PLRS_IMU_PROTOCOL.{h,c}` -- decoder (CRC, COBS, framing), host-tested under
  `test/`.
- `PLRS_IMU.{h,c}` -- STM32U5 HAL transport: circular DMA receive, drained by
  polling.

## Wiring

IMU TX -> MCU UART RX, 115200 8N1. Receive only. On the rudder controller this
is USART3.

## IOC: USART3

- Mode -> Asynchronous
- Parameter Settings -> Baud Rate = 115200, 8 bits, no parity, 1 stop bit
- NVIC Settings -> USART3 Global Interrupt = Enabled

## IOC: GPDMA1 (USART3_RX)

- Add a channel, Request = `GPDMA1_REQUEST_USART3_RX`
- Direction = Peripheral to Memory
- Mode = Circular
- Source (peripheral) Address Increment = Disabled
- Destination (memory) Address Increment = Enabled
- Source / Destination Data Width = Byte

## Usage

```c
static PLRS_IMU imu;

PLRS_IMU__init(&imu, &huart3);

// each main-loop pass:
PLRS_IMU__service(&imu);

float heading;
if (PLRS_IMU__isFresh(&imu, 200) && PLRS_IMU__getHeading(&imu, &heading)) {
    // steer on heading
} else {
    // fail safe: no recent heading
}
```

`PLRS_IMU__service` restarts reception if a UART error aborts the DMA.
`PLRS_IMU__drops` reports frames missed, counted from sequence-number gaps.
