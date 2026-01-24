CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O0

ROOT_DIR := ..
STUB_DIR := $(ROOT_DIR)/tests/stubs
COMMON_DIR := $(ROOT_DIR)/tests/common
NMEA_DIR := $(ROOT_DIR)/drv-modules/nmea0183
CANFD_DIR := $(ROOT_DIR)/com-modules/CANFD
BUILD_DIR := $(ROOT_DIR)/tests/build

NMEA_TEST := $(BUILD_DIR)/nmea0183_test
WIND_TEST := $(BUILD_DIR)/wind_sensor_test
AIS_TEST := $(BUILD_DIR)/ais_test
CANFD_TEST := $(BUILD_DIR)/canfd_test

.PHONY: all clean test

all: $(NMEA_TEST) $(WIND_TEST) $(AIS_TEST) $(CANFD_TEST)

$(NMEA_TEST): $(ROOT_DIR)/tests/drv-modules/nmea0183/nmea0183_test.c \
	$(ROOT_DIR)/tests/common/nmea_test_utils.c \
	$(NMEA_DIR)/NMEA0183.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(NMEA_DIR) $^ -o $@

$(WIND_TEST): $(ROOT_DIR)/tests/drv-modules/nmea0183/wind_sensor_test.c \
	$(ROOT_DIR)/tests/common/nmea_test_utils.c \
	$(NMEA_DIR)/NMEA0183.c \
	$(NMEA_DIR)/WIND_SENSOR.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(NMEA_DIR) -I$(CANFD_DIR) $^ -o $@

$(AIS_TEST): $(ROOT_DIR)/tests/drv-modules/nmea0183/ais_test.c \
	$(ROOT_DIR)/tests/common/nmea_test_utils.c \
	$(NMEA_DIR)/NMEA0183.c \
	$(NMEA_DIR)/AIS.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(NMEA_DIR) -I$(CANFD_DIR) $^ -o $@

$(CANFD_TEST): $(ROOT_DIR)/tests/com-modules/canfd/canfd_test.c \
	$(CANFD_DIR)/can.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(CANFD_DIR) $^ -o $@

test: all
	$(NMEA_TEST)
	$(WIND_TEST)
	$(AIS_TEST)
	$(CANFD_TEST)

clean:
	rm -rf $(BUILD_DIR)
