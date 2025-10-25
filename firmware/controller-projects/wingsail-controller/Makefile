# ------------------------------------------------------------
# Convenience targets for STM32CubeIDE projects (no IDE needed)
# ------------------------------------------------------------

PROJECT_ROOT ?= $(CURDIR)
BUILD_CONFIG ?= Debug
BUILD_DIR := $(PROJECT_ROOT)/$(BUILD_CONFIG)
TARGET_NAME ?= $(notdir $(abspath $(PROJECT_ROOT)))
ELF ?= $(BUILD_DIR)/$(TARGET_NAME).elf
BIN ?= $(BUILD_DIR)/$(TARGET_NAME).bin
OPENOCD_CFG ?= $(PROJECT_ROOT)/$(TARGET_NAME).cfg

MAKE_JOBS ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
OBJCOPY ?= arm-none-eabi-objcopy
BEAR ?= bear

# ------------------------------------------------------------
# Tool detection
# ------------------------------------------------------------

# Prefer system OpenOCD if available (avoids CubeIDE recursion bug)
SYSTEM_OPENOCD_SCRIPTS := $(wildcard /usr/share/openocd/scripts)
CUBEIDE_SCRIPTS := $(firstword $(wildcard /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.debug.openocd_*/resources/openocd/st_scripts))

ifeq ($(SYSTEM_OPENOCD_SCRIPTS),)
  ifeq ($(CUBEIDE_SCRIPTS),)
    $(warning No OpenOCD scripts found — flash/debug may fail.)
    OPENOCD_SCRIPT_DIR :=
  else
    OPENOCD_SCRIPT_DIR := $(CUBEIDE_SCRIPTS)
  endif
else
  OPENOCD_SCRIPT_DIR := $(SYSTEM_OPENOCD_SCRIPTS)
endif

OPENOCD ?= openocd
OPENOCD_S_ARG := $(if $(OPENOCD_SCRIPT_DIR),-s $(OPENOCD_SCRIPT_DIR),)

# Pick first available GDB
GDB_DEFAULT := $(firstword $(shell command -v gdb-multiarch 2>/dev/null))
ifeq ($(GDB_DEFAULT),)
GDB_DEFAULT := $(firstword $(shell command -v arm-none-eabi-gdb 2>/dev/null))
endif
GDB ?= $(if $(GDB_DEFAULT),$(GDB_DEFAULT),gdb-multiarch)

# ------------------------------------------------------------
# Output control
# ------------------------------------------------------------

QUIET ?= 0
ifneq ($(findstring s,$(MAKEFLAGS)),)
  QUIET := 1
endif
MAKE_SILENT := $(if $(filter 1,$(QUIET)),-s,)
QUIET_FILTER := sed -e '/^Finished building/d' -e '/^ *$$/d' -e '/^ *text[[:space:]]/d' -e '/^ *[0-9]/d'

.PHONY: help build clean rebuild compile_commands flash debug-server gdb bin sanitize_flags
.DEFAULT_GOAL := help

# ------------------------------------------------------------
# Targets
# ------------------------------------------------------------

help:
	@echo "Available targets:"
	@echo "  build             Build firmware using $(BUILD_CONFIG)/makefile"
	@echo "  clean             Clean build artifacts"
	@echo "  rebuild           Clean then build"
	@echo "  compile_commands  Regenerate compile_commands.json with bear"
	@echo "  bin               Create $(BIN)"
	@echo "  flash             Flash ELF with OpenOCD (builds first)"
	@echo "  debug-server      Start OpenOCD GDB server"
	@echo "  gdb               Launch arm-none-eabi-gdb and connect to server"
	@echo ""
	@echo "Detected OpenOCD script path: $(OPENOCD_SCRIPT_DIR)"
	@echo "Options:"
	@echo "  QUIET=1 or make -s   Suppress sub-make chatter"
	@echo "  BUILD_CONFIG=Release Use alternate configuration"
	@echo "  OPENOCD_CFG=foo.cfg  Use a different OpenOCD config"

# ------------------------------------------------------------

build: sanitize_flags
ifeq ($(QUIET),1)
	@bash -o pipefail -c "$(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) main-build -j$(MAKE_JOBS) 2>&1 | $(QUIET_FILTER)"
else
	$(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) main-build -j$(MAKE_JOBS)
endif

clean:
	$(MAKE) $(MAKE_SILENT) -C $(BUILD_DIR) clean

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

# ------------------------------------------------------------
# Binary + Flash
# ------------------------------------------------------------

bin: build
	$(OBJCOPY) -O binary $(ELF) $(BIN)
	@echo "Created $(BIN)"

flash: build
	@test -f $(ELF) || (echo "Missing ELF at $(ELF)." >&2; exit 1)
	@if [ -z "$(OPENOCD_SCRIPT_DIR)" ]; then \
		echo 'Warning: No OpenOCD scripts found. Check installation.'; \
	fi
	$(OPENOCD) $(OPENOCD_S_ARG) -f $(OPENOCD_CFG) -c "program $(ELF) verify reset exit"

debug-server:
	$(OPENOCD) $(OPENOCD_S_ARG) -f $(OPENOCD_CFG)

gdb: build
	$(GDB) $(ELF) -ex "target remote localhost:3333"
