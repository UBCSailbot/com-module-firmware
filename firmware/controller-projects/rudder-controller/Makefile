SHELL := /bin/sh

PROJECT ?= rudder-controller
BUILD_DIR ?= Debug
ELF ?= $(BUILD_DIR)/$(PROJECT).elf

OPENOCD ?= openocd
OPENOCD_INTERFACE ?= interface/stlink.cfg
OPENOCD_TARGET ?= target/stm32u5x.cfg
OPENOCD_PROGRAM_CMD ?= program $(ELF) verify reset exit
VM_NAME ?= ubuntu24.04
VIRSH ?= virsh -c qemu:///system
STLINK_HOSTDEV_XML ?= stlink-hostdev.xml

.PHONY: flash attach-stlink detach-stlink status-stlink

flash:
	@test -f "$(ELF)" || { echo "Missing ELF: $(ELF)"; exit 1; }
	$(OPENOCD) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) \
		-c "$(OPENOCD_PROGRAM_CMD)"

attach-stlink:
	@test -f "$(STLINK_HOSTDEV_XML)" || { echo "Missing XML: $(STLINK_HOSTDEV_XML)"; exit 1; }
	$(VIRSH) attach-device $(VM_NAME) $(STLINK_HOSTDEV_XML) --live --config

detach-stlink:
	@test -f "$(STLINK_HOSTDEV_XML)" || { echo "Missing XML: $(STLINK_HOSTDEV_XML)"; exit 1; }
	$(VIRSH) detach-device $(VM_NAME) $(STLINK_HOSTDEV_XML) --live --config

status-stlink:
	$(VIRSH) dumpxml $(VM_NAME) | rg -n "hostdev|0483|374e" || true
