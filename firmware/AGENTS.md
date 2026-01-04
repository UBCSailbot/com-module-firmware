# Repository Guidelines

## Project Structure & Module Organization

This repository contains STM32 firmware modules and controller projects.

- `drv-modules/` and `pwr-modules/`: reusable driver/power modules (C sources + headers).
- `controller-projects/`: board-specific projects, each with its own build directory and Makefile.
- `tests/`: host-based unit tests and stubs (e.g., `tests/nmea0183_test.c`, `tests/stubs/`).
- `compile_commands.json`: top-level compilation database (often symlinked by project build tooling).

## Build, Test, and Development Commands

Common commands (run from the repository root):

- `make -C controller-projects/wingsail-controller build`  
  Builds the Wingsail controller firmware using its generated build system.
- `make -C controller-projects/wingsail-controller compile_commands`  
  Regenerates `compile_commands.json` via `bear` for IDE tooling.
- `make -C tests`  
  Builds host-side tests (no STM32 toolchain required).
- `./tests/build/nmea0183_test`  
  Runs the NMEA0183 unit tests.

## Coding Style & Naming Conventions

- Language: C (firmware modules and tests).
- Follow existing file conventions (indentation, brace style, and spacing) for consistency.
- Naming patterns include `Module__function` for public APIs and uppercase macros (e.g., `MAX_*`, `MESSAGE_*`).
- Keep headers self-contained and prefer explicit `stdint.h` types.
- In `tests/`, use Doxygen comments with a blank line after `@brief`, and include `@param`/`@return` tags.

## Testing Guidelines

- Framework: minimal custom C test runner in `tests/`.
- Naming: `*_test.c` with a single `main()` for each test binary.
- Use representative, small NMEA samples (see `drv-modules/nmea0183/nmea-sample.txt`) rather than large fixtures.
- When adding tests, update `tests/Makefile` with the new target.

## Commit & Pull Request Guidelines

- Recent commits use short, imperative, capitalized subjects (e.g., “Add …”, “Update …”).
- Keep commits focused and include test results when relevant.
- PRs should describe what changed, why, and how it was validated (commands + outputs if applicable).

## Notes for Contributors

- Host tests rely on stub HAL headers in `tests/stubs/`; extend those stubs as needed for new modules.
- Do not modify generated build artifacts under controller `Debug/` or `Release/` folders.
