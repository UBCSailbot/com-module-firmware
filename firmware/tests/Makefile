CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O0

ROOT_DIR := ..
STUB_DIR := $(ROOT_DIR)/tests/stubs
NMEA_DIR := $(ROOT_DIR)/drv-modules/nmea0183
BUILD_DIR := $(ROOT_DIR)/tests/build

TARGET := $(BUILD_DIR)/nmea0183_test

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(ROOT_DIR)/tests/nmea0183_test.c $(NMEA_DIR)/NMEA0183.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(NMEA_DIR) $^ -o $@

clean:
	rm -rf $(BUILD_DIR)
