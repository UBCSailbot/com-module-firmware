#ifndef DCAN500_BASIC_CONFIG_H
#define DCAN500_BASIC_CONFIG_H

#include "main.h"
#include "can.h"
#include <stdint.h>
#include <stdbool.h>


//Register map
#define DCAN500_REG_1_DEVICE_CTRL1        0x01
#define DCAN500_REG_2_FREQ_SELECT         0x02
#define DCAN500_REG_3_SLEEP_IO_CTRL       0x03
#define DCAN500_REG_5_RXFIFO_THR_LSB      0x05
#define DCAN500_REG_6_RXFIFO_THR_MSB      0x06

#define DCAN500_REG_9_BITTIME_SEG1_LSB    0x09
#define DCAN500_REG_B_BITTIME_SEG1_MSB    0x0B
#define DCAN500_REG_C_BITTIME_SEG2_LSB    0x0C
#define DCAN500_REG_E_BITTIME_SEG2_MSB    0x0E


//Command protocol
#define DCAN500_CMD_CAN_ID                0x555U
#define DCAN500_READBACK_CAN_ID           0x000U

#define DCAN500_WRITE_REG_CMD             0xF5U
#define DCAN500_READ_REG_CMD              0xFDU


//Register 1
#define DCAN500_REG1_TX_HIGH_POWER        (1U << 0)   /* 0=33mA, 1=66mA */
#define DCAN500_REG1_TX_LEVEL_2VPP        (1U << 3)   /* 0=1Vpp, 1=2Vpp */
#define DCAN500_REG1_FIXED_BITS           0xF0U       /* set bits[7:4] = 1111 */


//Register 3
#define DCAN500_REG3_SLEEP_MODE_SLP1      0x00U
#define DCAN500_REG3_SLEEP_MODE_SLP2      0x01U
#define DCAN500_REG3_SLEEP_MODE_SLP3      0x02U
#define DCAN500_REG3_SLEEP_MODE_SLP4      0x03U
#define DCAN500_REG3_LONG_WUM             (1U << 2)
#define DCAN500_REG3_AUTO_WUM             (1U << 3)
#define DCAN500_REG3_ENTER_SLEEP          (1U << 7)


// Configuration setting menu
typedef struct
{
    float carrier_freq_mhz;         /* 5.0 ~ 30.0, 100kHz step */
    bool tx_high_power;             /* REG_1 bit0 */
    bool tx_level_2vpp;             /* REG_1 bit3 */

    uint16_t rxfifo_almost_full;    /* REG_4 + REG_5, 10-bit threshold: 0~1023 */

    bool configure_sleep_reg3;      /* Configure REG_3 or skip */
    uint8_t reg3_value;

    bool configure_1mbit;            
    uint8_t reg9_value;             /* Prop_Seg + Phase_Seg1 */
    uint8_t regb_value;             /* Phase_Seg2 */  
    uint8_t regc_value;             /* SJW */
    uint8_t rege_value;             /* Prescaler */ 
} DCAN500_Config_t;


//Public API
HAL_StatusTypeDef DCAN500_WriteRegister(FDCAN_HandleTypeDef *hfdcan, uint8_t reg, uint8_t value);
HAL_StatusTypeDef DCAN500_ReadRegister(FDCAN_HandleTypeDef *hfdcan, uint8_t reg, uint8_t *value);

HAL_StatusTypeDef DCAN500_ApplyConfig(FDCAN_HandleTypeDef *hfdcan, const DCAN500_Config_t *cfg);

/* Quick setup for 500k or 1M */
HAL_StatusTypeDef DCAN500_ConfigDefault500k(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef DCAN500_Config1M(FDCAN_HandleTypeDef *hfdcan);

/* Frequency Translate */
uint8_t DCAN500_CarrierFreqToReg(float freq_mhz);

#endif












