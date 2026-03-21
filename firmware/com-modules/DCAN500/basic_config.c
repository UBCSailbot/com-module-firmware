/*
* DCAN500 Basic Configuration
*
* This file provides basic configuration and register access for the DCAN500 CAN-over-powerline transceiver using CAN frames.
*
* Structure:
*
* 1. Command Mode Control
* Enter/exit command mode using HDC pin
*
* 2. Other Helper Function
* Carrier frequency conversion (MHz -> REG_2)
* 
* 3. Register access
* DCAN500_WriteRegister(): send write command (padded to 8 bytes)
* DCAN500_ReadRegister(): send read command and wait for response
*
* 4. Configuration
* * DCAN500_ApplyConfig(): apply all register settings (Enter command mode -> write register -> exit)
*
* 5. 500k / 1M quick setting
* * DCAN500_ConfigDefault500k()
* * DCAN500_Config1M()
*
* Notes:
* * Uses CAN_Transmit / CAN_Receive from can.c
* * Frames are padded to 8 bytes for 1M compatibility
* * REG_9 ~ REG_E must be updated to match CAN bit timing
* 
* can.c has to include:
* * BUS_BUSY or RTR handling
* * Bite rate check (For DLC constraints check)
*/


#include "basic_config.h"
#include <math.h>
#include <string.h>

//CHANGE THESE TO YOUR GPIO!!
#define DCAN500_HDC_GPIO_Port   GPIOA
#define DCAN500_HDC_Pin         GPIO_PIN_0

//Delay helper
static void DCAN500_CommandDelay(void)
{
    /* datasheet asks for >=100ns */
    for (volatile uint32_t i = 0; i < 1000; i++);
}


//Low HDC, enter command mode
static void DCAN500_EnterCommandMode(void)
{
    HAL_GPIO_WritePin(DCAN500_HDC_GPIO_Port, DCAN500_HDC_Pin, GPIO_PIN_RESET);
    DCAN500_CommandDelay();
}


//High HDC, back to normal operation mode
static void DCAN500_ExitCommandMode(void)
{
    DCAN500_CommandDelay();
    HAL_GPIO_WritePin(DCAN500_HDC_GPIO_Port, DCAN500_HDC_Pin, GPIO_PIN_SET);
    DCAN500_CommandDelay();
}


//Translate required frequency to REG_2 value
uint8_t DCAN500_CarrierFreqToReg(float freq_mhz)
{
    // Frequency 5 - 30
    if (freq_mhz < 5.0f)  freq_mhz = 5.0f;
    if (freq_mhz > 30.0f) freq_mhz = 30.0f;

    float raw = (freq_mhz - 5.0f) * 10.0f;
    int reg = (int)lroundf(raw);

    //REG_2 value 0 - 250
    if (reg < 0) reg = 0;
    if (reg > 250) reg = 250;

    return (uint8_t)reg;
}

//Write a rigister command
HAL_StatusTypeDef DCAN500_WriteRegister(FDCAN_HandleTypeDef *hfdcan, uint8_t reg, uint8_t value)
{
    uint8_t data[8] = {
        DCAN500_WRITE_REG_CMD,
        reg,
        value,
        0,0,0,0,0 // padding zeros
    };

    DCAN500_EnterCommandMode();

    HAL_StatusTypeDef status = CAN_Transmit(
        DCAN500_CMD_CAN_ID,
        FDCAN_STANDARD_ID,
        FDCAN_DLC_BYTES_8,
        data,
        hfdcan
    );

    DCAN500_ExitCommandMode();
    HAL_Delay(1);
    return status;
}

//Read register
HAL_StatusTypeDef DCAN500_ReadRegister(FDCAN_HandleTypeDef *hfdcan, uint8_t reg, uint8_t *value)
{
    if (value == NULL) return HAL_ERROR;

    uint8_t cmd[8] = {
        DCAN500_READ_REG_CMD,
        reg,
        0,0,0,0,0,0 //padding zeros
    };

    CAN_Frame frame;
    uint32_t start = HAL_GetTick();

    DCAN500_EnterCommandMode();

    HAL_StatusTypeDef status = CAN_Transmit(
        DCAN500_CMD_CAN_ID,
        FDCAN_STANDARD_ID,
        FDCAN_DLC_BYTES_8,
        cmd,
        hfdcan
    );

    if (status != HAL_OK)
    {
        DCAN500_ExitCommandMode();
        return status;
    }

    while ((HAL_GetTick() - start) < 50U)
    {
        if (CAN_Receive(&frame) == HAL_OK)
        {
            if (frame.RxData1_Identifier != DCAN500_READBACK_CAN_ID)
                continue;

            if (frame.RxData1_BufferLength != 1U)
                continue;

            *value = frame.RxData1[0];
            DCAN500_ExitCommandMode();
            return HAL_OK;
        }
    }

    DCAN500_ExitCommandMode();
    return HAL_TIMEOUT;
}
  
  DCAN500_ExitCommandMode();
  return HAL_TIMEOUT;
}


HAL_StatusTypeDef DCAN500_ApplyConfig(FDCAN_HandleTypeDef *hfdcan, const DCAN500_Config_t *cfg) 
{ 

    if ((hfdcan == NULL) || (cfg == NULL)) return HAL_ERROR; 
    HAL_StatusTypeDef status; 

    /* REG_1 */ 
    uint8_t reg1 = DCAN500_REG1_FIXED_BITS; 
    if (cfg->tx_high_power) reg1 |= DCAN500_REG1_TX_HIGH_POWER; 
    if (cfg->tx_level_2vpp) reg1 |= DCAN500_REG1_TX_LEVEL_2VPP; 

    //status check
    status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_1_DEVICE_CTRL1, reg1); 
    if (status != HAL_OK) return status; 

    
    /* REG_2 : carrier frequency */ 
    uint8_t reg2 = DCAN500_CarrierFreqToReg(cfg->carrier_freq_mhz); 

    status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_2_FREQ_SELECT, reg2); 
    if (status != HAL_OK) return status; 

    HAL_Delay(1);

    
    /* REG_5 / REG_6 : RX FIFO threshold */ 
    uint16_t thr = (cfg->rxfifo_almost_full & 0x03FFU); 
    uint8_t reg5 = (uint8_t)(thr & 0xFFU); 
    uint8_t reg6 = (uint8_t)((thr >> 8) & 0x03U); 

    status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_5_RXFIFO_THR_LSB, reg5); 
    if (status != HAL_OK) return status; 
    status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_6_RXFIFO_THR_MSB, reg6); 
    if (status != HAL_OK) return status; 


    /* REG_3 (optional) */ 
    if (cfg->configure_sleep_reg3) 
    { 
        status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_3_SLEEP_IO_CTRL, cfg->reg3_value); 
        if (status != HAL_OK) return status; 
    } 

 
    /* 1M bitrate set */ 
    if (cfg->configure_1mbit) 
    { 
        status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_9_BITTIME_SEG1_LSB, cfg->reg9_value); 
        if (status != HAL_OK) return status; 
        HAL_Delay(1);
        
        status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_B_BITTIME_SEG1_MSB, cfg->regb_value); 
        if (status != HAL_OK) return status; 
        HAL_Delay(1);
        
        status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_C_BITTIME_SEG2_LSB, cfg->regc_value); 
        if (status != HAL_OK) return status; 
        HAL_Delay(1);
        
        status = DCAN500_WriteRegister(hfdcan, DCAN500_REG_E_BITTIME_SEG2_MSB, cfg->rege_value); 
        if (status != HAL_OK) return status; 
        HAL_Delay(1);
    } 

    return HAL_OK; 
} 

 
//500k quick setup
HAL_StatusTypeDef DCAN500_ConfigDefault500k(FDCAN_HandleTypeDef *hfdcan) 
{ 

    DCAN500_Config_t cfg; 

    memset(&cfg, 0, sizeof(cfg)); 

    cfg.carrier_freq_mhz      = 13.0f;   
    cfg.tx_high_power         = false;   //33mA
    cfg.tx_level_2vpp         = true;    
    cfg.rxfifo_almost_full    = 256;     
    cfg.configure_sleep_reg3  = true; 
    cfg.reg3_value            = 0x08;    //turn on wake up mode
    cfg.configure_1mbit       = false;   //500k by BR_SEL pins

    return DCAN500_ApplyConfig(hfdcan, &cfg);
} 

//1M quick setup
HAL_StatusTypeDef DCAN500_Config1M(FDCAN_HandleTypeDef *hfdcan) 
{ 
    DCAN500_Config_t cfg; 

    memset(&cfg, 0, sizeof(cfg)); 

    cfg.carrier_freq_mhz      = 13.0f; 
    cfg.tx_high_power         = false; 
    cfg.tx_level_2vpp         = true; 
    cfg.rxfifo_almost_full    = 256; 
    cfg.configure_sleep_reg3  = true; 
    cfg.reg3_value            = 0x08;    //wake up mode on
    cfg.configure_1mbit       = true; 

    /* need to be changed after get actual mcu CAN timing*/
    /* 
    cfg.reg9_value            = 0x40; 
    cfg.regb_value            = 0x00; 
    cfg.regc_value            = 0x10; 
    cfg.rege_value            = 0x00; 
    */

    return DCAN500_ApplyConfig(hfdcan, &cfg); 
} 

 















