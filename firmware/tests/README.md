# Unit Tests (Host)

This folder contains a lightweight, host-built test harness for firmware modules.
It uses a minimal C test runner (no external framework) and stub HAL headers so
the modules can compile on a developer machine.

## Build and run

From the firmware root:

```sh
make -C tests
./tests/build/nmea0183_test
```

To build and run all tests:

```sh
make -C tests test
```

## What is covered

- NMEA0183 parsing and validation in `drv-modules/nmea0183/NMEA0183.c`
- Sample sentences derived from `drv-modules/nmea0183/nmea-sample.txt`
- Ring buffer overflow behavior via the IRQ handler path
- Common failure modes (bad start, bad checksum, bad termination, short receive)

## Stubs

The host build uses stub headers in `tests/stubs` to avoid pulling in STM32 HAL
dependencies. These provide minimal types/macros for:

- `stm32u5xx_hal.h` (UART/DMA definitions + HAL calls)
- `main.h` (`Error_Handler`)

If you add new module tests that require additional HAL symbols, extend the stubs
locally rather than importing the full device headers.

## Adding new tests

- Add a new `*_test.c` under `tests/drv-modules/...` to mirror the module path.
- Reuse helpers in `tests/common/nmea_test_utils.c` where applicable.
- Update `tests/Makefile` to build the new target
- Keep test data small and focused; prefer a few representative NMEA sentences
  instead of the full sample file
