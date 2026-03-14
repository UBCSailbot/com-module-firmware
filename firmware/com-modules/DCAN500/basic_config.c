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
    for (volatile uint32_t i = 0; i < 100; i++) { __NOP(); }
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

  //command frame payload
  uint8_t data[3] = {
        DCAN500_WRITE_REG_CMD,
        reg,
        value
    };

    DCAN500_EnterCommandMode();

    HAL_StatusTypeDef status = CAN_Transmit(
        DCAN500_CMD_CAN_ID,
        FDCAN_STANDARD_ID,
        FDCAN_DLC_BYTES_3,
        data,
        hfdcan
    );

    DCAN500_ExitCommandMode();
    return status;
}


//Read register
HAL_StatusTypeDef DCAN500_ReadRegister(FDCAN_HandleTypeDef *hfdcan, uint8_t reg, uint8_t *value)
{
    if (value == NULL) return HAL_ERROR;

    uint8_t cmd[2] = {
        DCAN500_READ_REG_CMD,
        reg
    };

    CAN_Frame frame;
    uint32_t start = HAL_GetTick();

    DCAN500_EnterCommandMode();

    HAL_StatusTypeDef status = CAN_Transmit(
        DCAN500_CMD_CAN_ID,
        FDCAN_STANDARD_ID,
        FDCAN_DLC_BYTES_2,
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
            /* check for response ID, and length*/
            if ((frame.RxData1_Identifier == DCAN500_READBACK_CAN_ID) &&
                (frame.RxData1_BufferLength >= 1U))
            {
                *value = frame.RxData1[0];
                DCAN500_ExitCommandMode();
                return HAL_OK;
            }
        }
    }
  
  DCAN500_ExitCommandMode();
  return HAL_TIMEOUT;
}
















