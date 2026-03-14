#include "can.h"

extern FDCAN_HandleTypeDef hfdcan1;

void DCAN500_ConfigExample(void)
{
    /* Initialize CAN library */
    CAN_Init(&hfdcan1, 0x130);

    uint8_t data[3] = {0x01, 0x02, 0x03};

    /* Send test message */
    CAN_Transmit(
        0x040,
        FDCAN_STANDARD_ID,
        FDCAN_DLC_BYTES_3,
        data,
        &hfdcan1
    );
}


