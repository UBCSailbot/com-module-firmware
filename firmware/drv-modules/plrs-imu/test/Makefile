CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined

CORE := ../PLRS_IMU_PROTOCOL.c
HDR := ../PLRS_IMU_PROTOCOL.h
BIN := test_plrs_imu_protocol

test: $(BIN)
	./$(BIN)

$(BIN): test_plrs_imu_protocol.c $(CORE) $(HDR)
	$(CC) $(CFLAGS) -I.. -o $@ test_plrs_imu_protocol.c $(CORE)

clean:
	rm -f $(BIN)

.PHONY: test clean
