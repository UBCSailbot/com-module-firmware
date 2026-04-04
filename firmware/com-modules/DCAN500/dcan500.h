/*
 * dcan500.h — DCAN500 Register Definitions & API
 * 
 * Created on:
 * Author: 
 *
 * @brief: Header file for DCAN500 Library
 *
 * @details: This file contains function prototypes and global variables for configuring 
 *            the DCAN500 chip.
 */

#ifndef DCAN500_H
#define DCAN500_H

#include "main.h"
#include "can.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Register Addresses ---- */
#define DCAN500_REG_1_DEVICE_CTRL1        0x01
#define DCAN500_REG_2_FREQ_SELECT         0x02
#define DCAN500_REG_3_SLEEP_IO_CTRL       0x03
#define DCAN500_REG_5_RXFIFO_THR_LSB      0x05
#define DCAN500_REG_6_RXFIFO_THR_MSB      0x06
#define DCAN500_REG_9_BITTIME_SEG1_LSB    0x09
#define DCAN500_REG_B_BITTIME_SEG1_MSB    0x0B
#define DCAN500_REG_C_BITTIME_SEG2_LSB    0x0C
#define DCAN500_REG_E_BITTIME_SEG2_MSB    0x0E

/* ---- Command Protocol ---- */
#define DCAN500_CMD_CAN_ID                0x555U
#define DCAN500_READBACK_CAN_ID           0x000U
#define DCAN500_WRITE_REG_CMD             0xF5U
#define DCAN500_READ_REG_CMD              0xFDU

/* ---- REG_1 Bit Definitions ---- */
#define DCAN500_REG1_TX_HIGH_POWER        (1U << 0)   /* 0=33mA, 1=66mA */
#define DCAN500_REG1_TX_LEVEL_2VPP        (1U << 3)   /* 0=1Vpp, 1=2Vpp */
#define DCAN500_REG1_FIXED_BITS           0xF0U       /* bits[7:4] = 1111 */

/* ---- REG_3 Bit Definitions ---- */
#define DCAN500_REG3_SLEEP_MODE_SLP1      0x00U
#define DCAN500_REG3_SLEEP_MODE_SLP2      0x01U
#define DCAN500_REG3_SLEEP_MODE_SLP3      0x02U
#define DCAN500_REG3_SLEEP_MODE_SLP4      0x03U
#define DCAN500_REG3_LONG_WUM             (1U << 2)
#define DCAN500_REG3_AUTO_WUM             (1U << 3)
#define DCAN500_REG3_ENTER_SLEEP          (1U << 7)

/* ---- API ---- */

/* Command mode control */
void DCAN500_Enter_Command_Mode(void);
void DCAN500_Exit_Command_Mode(void);

/* Register access (caller must be in command mode) */
void DCAN500_WriteReg(uint8_t addr, uint8_t data);
void DCAN500_ReadReg(CAN_Frame *response, uint8_t addr);

/* Frequency helper */
uint8_t DCAN500_CarrierFreqToReg(float freq_mhz);

/* Quick setup (handles command mode internally) */
void DCAN500_Init_500k(float carrier_freq_MHz);
void DCAN500_Init_1M(float carrier_freq_MHz);

#endif
