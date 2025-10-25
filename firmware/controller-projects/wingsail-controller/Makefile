# Convenience targets for building and flashing STM32CubeIDE projects without CubeIDE.

PROJECT_ROOT ?= $(CURDIR)
BUILD_CONFIG ?= Debug
BUILD_DIR := $(PROJECT_ROOT)/$(BUILD_CONFIG)
TARGET_NAME ?= $(notdir $(abspath $(PROJECT_ROOT)))
ELF ?= $(BUILD_DIR)/$(TARGET_NAME).elf
BIN ?= $(BUILD_DIR)/$(TARGET_NAME).bin
OPENOCD_CFG ?= $(PROJECT_ROOT)/$(TARGET_NAME).cfg

MAKE_JOBS ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
OPENOCD ?= openocd
CUBEIDE_SCRIPTS := $(firstword $(wildcard /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.debug.openocd_*/resources/openocd/st_scripts))
OPENOCD_SCRIPT_DIR ?= $(CUBEIDE_SCRIPTS)
OPENOCD_S_ARG := $(if $(OPENOCD_SCRIPT_DIR),-s $(OPENOCD_SCRIPT_DIR),)
GDB_DEFAULT := $(firstword $(shell command -v gdb-multiarch 2>/dev/null))
ifeq ($(GDB_DEFAULT),)
GDB_DEFAULT := $(firstword $(shell command -v arm-none-eabi-gdb 2>/dev/null))
endif
GDB ?= $(if $(GDB_DEFAULT),$(GDB_DEFAULT),gdb-multiarch)
OBJCOPY ?= arm-none-eabi-objcopy
BEAR ?= bear

QUIET ?= 0
ifneq ($(findstring s,$(MAKEFLAGS)),)
	QUIET := 1
endif
MAKE_SILENT := $(if $(filter 1,$(QUIET)),-s,)
QUIET_FILTER := sed -e '/^Finished building/d' -e '/^ *$$/d' -e '/^ *text[[:space:]]/d' -e '/^ *[0-9]/d'

.PHONY: help build clean rebuild compile_commands flash debug-server gdb bin sanitize_flags

.DEFAULT_GOAL := help

help:
	@echo "Available targets:"
	@echo "  build             Build firmware using $(BUILD_CONFIG)/makefile"
	@echo "  clean             Clean build artifacts"
	@echo "  rebuild           Clean then build"
	@echo "  compile_commands  Regenerate compile_commands.json with bear"
	@echo "  bin               Create $(BUILD_CONFIG)/$(TARGET_NAME).bin"
	@echo "  flash             Flash ELF with OpenOCD (builds first)"
	@echo "  debug-server      Start OpenOCD GDB server"
	@echo "  gdb               Launch arm-none-eabi-gdb and connect to server"
	@echo ""
	@echo "Options:"
	@echo "  QUIET=1 or make -s   Suppress sub-make chatter"
	@echo "  PROJECT_ROOT=/path   Override project directory"
	@echo "  BUILD_CONFIG=Release Use non-default CubeIDE configuration"
	@echo "  TARGET_NAME=name     Override firmware artifact name"
	@echo "  OPENOCD_CFG=foo.cfg  Use a different OpenOCD config"
	@echo "  OPENOCD_SCRIPT_DIR=/path/to/scripts  Override OpenOCD script search"

build: sanitize_flags
ifeq ($(QUIET),1)
	@bash -o pipefail -c "$(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) main-build -j$(MAKE_JOBS) 2>&1 | $(QUIET_FILTER)"
else
	$(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) main-build -j$(MAKE_JOBS)
endif

clean:
ifeq ($(QUIET),1)
	@$(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) clean
else
	$(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) clean
endif

rebuild: clean build

sanitize_flags:
	@find $(BUILD_DIR) -name subdir.mk -exec sed -i 's/ -fcyclomatic-complexity//g' {} +

compile_commands: sanitize_flags
ifeq ($(QUIET),1)
	@bash -o pipefail -c "$(BEAR) --output $(BUILD_DIR)/compile_commands.json -- $(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) main-build -j$(MAKE_JOBS) 2>&1 | $(QUIET_FILTER)"
else
	$(BEAR) --output $(BUILD_DIR)/compile_commands.json -- $(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) main-build -j$(MAKE_JOBS)
endif
	ln -sf $(BUILD_CONFIG)/compile_commands.json $(PROJECT_ROOT)/compile_commands.json

bin: build
	$(OBJCOPY) -O binary $(ELF) $(BIN)
	@echo "Created $(BIN)"

flash: build
	@test -f $(ELF) || (echo "Missing ELF at $(ELF)." >&2; exit 1)
	@if [ -z "$(OPENOCD_SCRIPT_DIR)" ]; then \
		echo 'Warning: OPENOCD_SCRIPT_DIR not set and STM32CubeIDE scripts not found; SWV helpers may be missing.'; \
	fi
	$(OPENOCD) $(OPENOCD_S_ARG) -f $(OPENOCD_CFG) -c "program $(ELF) verify reset exit"

debug-server:
ifeq ($(OPENOCD_SCRIPT_DIR),)
	@echo 'Warning: OPENOCD_SCRIPT_DIR not set and STM32CubeIDE scripts not found; SWV helpers may be missing.'
endif
	$(OPENOCD) $(OPENOCD_S_ARG) -f $(OPENOCD_CFG)

gdb: build
	$(GDB) $(ELF) -ex "target remote localhost:3333"
