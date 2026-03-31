/*
 * basic_config.c — DCAN500 CAN-over-Powerline Configuration
 *
 * DLC follows datasheet exactly:
 *   WRITE-REG: DLC=3  (Table 13: [0xF5][addr][data])
 *   READ-REG:  DLC=2  (Table 14: [0xFD][addr])
 *
 * Command mode is NOT entered/exited inside WriteReg/ReadReg.
 * Caller manages command mode, or use the Init functions which handle it.
 *
 * Dependencies: can.c (CAN_Transmit, CAN_Receive)
 */

#include "basic_config.h"
#include <math.h>

/* ---- GPIO for HDC pin (CHANGE TO YOUR HARDWARE) ---- */
#define DCAN500_HDC_GPIO_Port   GPIOB
#define DCAN500_HDC_Pin         GPIO_PIN_0

extern FDCAN_HandleTypeDef hfdcan1;


/* ===========================================================
 *  Command Mode Control
 * =========================================================== */

void DCAN500_Enter_Command_Mode(void)
{
    HAL_GPIO_WritePin(DCAN500_HDC_GPIO_Port, DCAN500_HDC_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);  /* Datasheet: >= 100ns delay */
}

void DCAN500_Exit_Command_Mode(void)
{
    HAL_Delay(1);
    HAL_GPIO_WritePin(DCAN500_HDC_GPIO_Port, DCAN500_HDC_Pin, GPIO_PIN_SET);
}


/* ===========================================================
 *  Register Access (caller must be in command mode)
 * =========================================================== */

/* WRITE-REG: ID=0x555, DLC=3, Data=[0xF5, addr, data] */
void DCAN500_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t buffer[3] = { DCAN500_WRITE_REG_CMD, addr, data };
    if (CAN_Transmit(DCAN500_CMD_CAN_ID, FDCAN_DLC_BYTES_3,
                     buffer, &hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(1);
}

/* READ-REG: ID=0x555, DLC=2, Data=[0xFD, addr]
 * Response:  ID=0x000, DLC=1, Data=[value]            */
void DCAN500_ReadReg(CAN_Frame *response, uint8_t addr)
{
    uint8_t buffer[2] = { DCAN500_READ_REG_CMD, addr };
    if (CAN_Transmit(DCAN500_CMD_CAN_ID, FDCAN_DLC_BYTES_2,
                     buffer, &hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(1);
    if (CAN_Receive(response) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ===========================================================
 *  Frequency Helper
 *
 *  Datasheet Equation 1: REG_2 = (freq_MHz - 5) * 10
 *  Range: 5 MHz ~ 30 MHz  ->  REG_2: 0 ~ 250
 * =========================================================== */

uint8_t DCAN500_CarrierFreqToReg(float freq_mhz)
{
    if (freq_mhz < 5.0f)  freq_mhz = 5.0f;
    if (freq_mhz > 30.0f) freq_mhz = 30.0f;

    int reg = (int)lroundf((freq_mhz - 5.0f) * 10.0f);
    if (reg < 0)   reg = 0;
    if (reg > 250) reg = 250;

    return (uint8_t)reg;
}


/* ===========================================================
 *  Init: 500kbit/s
 *
 *  - Carrier frequency: user-specified
 *  - TX: 33mA, 2Vpp
 *  - RX-FIFO threshold: 256 bytes
 *  - Wake-up mode enabled (REG_3 = 0x2C)
 *  - Bitrate by BR_SEL pins (set to '11' for 500k)
 *
 *  Note: DLC >= 3 required at 500kbit/s (Table 20)
 * =========================================================== */

void DCAN500_Init_500k(float carrier_freq_MHz)
{
    DCAN500_Enter_Command_Mode();

    /* REG_1: TX level 2Vpp, 33mA drive */
    DCAN500_WriteReg(DCAN500_REG_1_DEVICE_CTRL1,
                     DCAN500_REG1_FIXED_BITS | DCAN500_REG1_TX_LEVEL_2VPP);

    /* REG_2: Carrier frequency */
    DCAN500_WriteReg(DCAN500_REG_2_FREQ_SELECT,
                     DCAN500_CarrierFreqToReg(carrier_freq_MHz));

    /* REG_5 + REG_6: RX-FIFO threshold = 256 */
    DCAN500_WriteReg(DCAN500_REG_5_RXFIFO_THR_LSB, 0x00);  /* lower 8 bits */
    DCAN500_WriteReg(DCAN500_REG_6_RXFIFO_THR_MSB, 0x01);  /* upper 2 bits */

    /* REG_3: Sleep & IO Control
     * Datasheet Section 5.3, default = 0x2C (bits: 00101100)
     *   bit[7]   = 0  (enter sleep, auto-clears)
     *   bit[6]   = 0  (fixed)
     *   bit[5]   = 1  (fixed, must stay 1)
     *   bit[4]   = 0  (fixed)
     *   bit[3]   = 1  (auto WUM enabled)
     *   bit[2]   = 1  (long WUM, default)
     *   bit[1:0] = 00 (SLP1, enhanced sleep)
     * Result: 0x2C
     */
    DCAN500_WriteReg(DCAN500_REG_3_SLEEP_IO_CTRL, 0x2C);

    DCAN500_Exit_Command_Mode();
    HAL_Delay(2);  /* Wait for carrier frequency to settle */
}


/* ===========================================================
 *  Init: 1Mbit/s
 *
 *  Same analog setup as 500k, plus bit timing registers.
 *
 *  Datasheet Example 3 (Section 11.1.2), 80% sample point:
 *    Total = (1/1000k) / 12.5ns = 80 samples
 *    Seg_1 = 64 = 0x040  ->  REG_9=0x40, REG_B=0x00
 *    Seg_2 = 16 = 0x010  ->  REG_C=0x10, REG_E=0x00
 *
 *  TODO: Verify these match your MCU CAN timing, then uncomment.
 *
 *  Note: DLC >= 7 required at 1Mbit/s (Table 20)
 * =========================================================== */

void DCAN500_Init_1M(float carrier_freq_MHz)
{
    DCAN500_Enter_Command_Mode();

    /* REG_1: TX level 2Vpp, 33mA drive */
    DCAN500_WriteReg(DCAN500_REG_1_DEVICE_CTRL1,
                     DCAN500_REG1_FIXED_BITS | DCAN500_REG1_TX_LEVEL_2VPP);

    /* REG_2: Carrier frequency */
    DCAN500_WriteReg(DCAN500_REG_2_FREQ_SELECT,
                     DCAN500_CarrierFreqToReg(carrier_freq_MHz));

    /* REG_5 + REG_6: RX-FIFO threshold = 256 */
    DCAN500_WriteReg(DCAN500_REG_5_RXFIFO_THR_LSB, 0x00);
    DCAN500_WriteReg(DCAN500_REG_6_RXFIFO_THR_MSB, 0x01);

    /* REG_3: Sleep & IO Control (same as 500k, see Section 5.3) */
    DCAN500_WriteReg(DCAN500_REG_3_SLEEP_IO_CTRL, 0x2C);

    /* Bit timing for 1Mbit/s — uncomment after verifying with MCU */
    /*
    DCAN500_WriteReg(DCAN500_REG_9_BITTIME_SEG1_LSB, 0x40);
    DCAN500_WriteReg(DCAN500_REG_B_BITTIME_SEG1_MSB, 0x00);
    DCAN500_WriteReg(DCAN500_REG_C_BITTIME_SEG2_LSB, 0x10);
    DCAN500_WriteReg(DCAN500_REG_E_BITTIME_SEG2_MSB, 0x00);
    */

    DCAN500_Exit_Command_Mode();
    HAL_Delay(2);
}
