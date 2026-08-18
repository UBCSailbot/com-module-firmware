CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O0

ROOT_DIR := ..
STUB_DIR := $(ROOT_DIR)/tests/stubs
COMMON_DIR := $(ROOT_DIR)/tests/common
NMEA_DIR := $(ROOT_DIR)/drv-modules/nmea0183
CANFD_DIR := $(ROOT_DIR)/com-modules/CANFD
RUDDER_DIR := $(ROOT_DIR)/drv-modules/rudder-control-model
BUILD_DIR := $(ROOT_DIR)/tests/build

NMEA_TEST := $(BUILD_DIR)/nmea0183_test
WIND_TEST := $(BUILD_DIR)/wind_sensor_test
AIS_TEST := $(BUILD_DIR)/ais_test
GPS_TEST := $(BUILD_DIR)/gps_test
CANFD_TEST := $(BUILD_DIR)/canfd_test
NMEA_SCHEDULER_TEST := $(BUILD_DIR)/nmea0183_scheduler_test
RUDDER_STRAIGHT_TEST := $(BUILD_DIR)/rudder_straight_test
RUDDER_SM_TEST := $(BUILD_DIR)/rudder_state_machine_test

# The rudder control model predates this harness and still carries unused-variable
# and const-qualifier warnings, so -Werror is dropped for its two targets only.
# Cleaning those up is separate work; do not copy this exemption elsewhere.
RUDDER_CFLAGS := -std=c11 -Wall -Wextra -O0 -DTEST_USE_FAKE_TICK -DRUDDER_QUIET
RUDDER_SRCS := $(RUDDER_DIR)/RUDDER.c $(RUDDER_DIR)/RUDDER_PARAMS.c \
	$(RUDDER_DIR)/ESTIMATOR.c $(RUDDER_DIR)/STATE_MACHINE.c

.PHONY: all clean test

all: $(NMEA_TEST) $(WIND_TEST) $(AIS_TEST) $(GPS_TEST) \
	$(NMEA_SCHEDULER_TEST) $(CANFD_TEST) \
	$(RUDDER_STRAIGHT_TEST) $(RUDDER_SM_TEST)

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

$(GPS_TEST): $(ROOT_DIR)/tests/drv-modules/nmea0183/gps_test.c \
	$(ROOT_DIR)/tests/common/nmea_test_utils.c \
	$(NMEA_DIR)/NMEA0183.c \
	$(NMEA_DIR)/GPS.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(NMEA_DIR) -I$(CANFD_DIR) $^ -o $@

$(NMEA_SCHEDULER_TEST): $(ROOT_DIR)/tests/drv-modules/nmea0183/nmea0183_scheduler_test.c \
	$(ROOT_DIR)/tests/common/nmea_test_utils.c \
	$(NMEA_DIR)/NMEA0183.c \
	$(NMEA_DIR)/AIS.c \
	$(NMEA_DIR)/GPS.c \
	$(NMEA_DIR)/WIND_SENSOR.c \
	$(NMEA_DIR)/NMEA0183_scheduler.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(NMEA_DIR) -I$(CANFD_DIR) $^ -o $@

$(CANFD_TEST): $(ROOT_DIR)/tests/com-modules/canfd/canfd_test.c \
	$(CANFD_DIR)/can.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(CANFD_DIR) $^ -o $@

$(RUDDER_STRAIGHT_TEST): $(ROOT_DIR)/tests/drv-modules/rudder-control-model/rudder_straight_test.c \
	$(ROOT_DIR)/tests/common/fake_tick.c \
	$(RUDDER_SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(RUDDER_CFLAGS) -I$(STUB_DIR) -I$(COMMON_DIR) -I$(RUDDER_DIR) $^ -o $@ -lm

# RUDDER_TEST_BUILD switches STRAIGHT_ONLY off so transitions can be exercised.
$(RUDDER_SM_TEST): $(ROOT_DIR)/tests/drv-modules/rudder-control-model/rudder_state_machine_test.c \
	$(ROOT_DIR)/tests/common/fake_tick.c \
	$(RUDDER_SRCS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(RUDDER_CFLAGS) -DRUDDER_TEST_BUILD -I$(STUB_DIR) -I$(COMMON_DIR) -I$(RUDDER_DIR) $^ -o $@ -lm

test: all
	$(NMEA_TEST)
	$(WIND_TEST)
	$(AIS_TEST)
	$(GPS_TEST)
	$(NMEA_SCHEDULER_TEST)
	$(CANFD_TEST)
	$(RUDDER_STRAIGHT_TEST)
	$(RUDDER_SM_TEST)

clean:
	rm -rf $(BUILD_DIR)
