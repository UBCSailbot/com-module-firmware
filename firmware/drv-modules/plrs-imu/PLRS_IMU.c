#include "PLRS_IMU.h"

void PLRS_IMU__init(PLRS_IMU *self, UART_HandleTypeDef *huart) {
    self->huart = huart;
    self->rx_read = 0;
    self->parser = (PlrsImuParser){0};
    self->has_heading = false;
    self->heading_deg = 0.0f;
    self->heading_ms = 0;
    self->has_attitude = false;
    self->heel_deg = 0.0f;
    self->yaw_rate_dps = 0.0f;
    self->attitude_ms = 0;
    self->last_rx_ms = 0;
    self->has_seq = false;
    self->last_seq = 0;
    self->drops = 0;
    self->heading_valid = false;
    HAL_UART_Receive_DMA(huart, self->rx_dma, sizeof(self->rx_dma));
}

static void PLRS_IMU__onFrame(PLRS_IMU *self, const PlrsImuFrame *frame) {
    if (self->has_seq) {
        self->drops += (uint8_t)(frame->seq - self->last_seq - 1);
    }
    self->last_seq = frame->seq;
    self->has_seq = true;
    self->last_rx_ms = HAL_GetTick();

    switch (frame->msg_id) {
    case PLRS_IMU_MSG_HEADING: {
        float deg;
        if (plrs_imu_heading_from_payload(frame->payload, frame->payload_len,
                                          &deg)) {
            self->heading_deg = deg;
            self->has_heading = true;
            self->heading_ms = HAL_GetTick();
        }
        break;
    }
    case PLRS_IMU_MSG_ATTITUDE: {
        PlrsImuAttitude att;
        if (plrs_imu_attitude_from_payload(frame->payload, frame->payload_len,
                                           &att)) {
            const uint32_t now = HAL_GetTick();
            self->heading_deg = att.heading_deg;
            self->has_heading = true;
            self->heading_ms = now;
            self->heel_deg = att.roll_deg;
            self->yaw_rate_dps = att.yaw_rate_dps;
            self->has_attitude = true;
            self->attitude_ms = now;
            self->heading_valid = att.heading_valid;
        }
        break;
    }
    default:
        break;
    }
}

void PLRS_IMU__service(PLRS_IMU *self) {
    // Restart reception if a UART error aborted the circular DMA.
    if (self->huart->RxState != HAL_UART_STATE_BUSY_RX) {
        self->rx_read = 0;
        plrs_imu_parser_reset(&self->parser);
        HAL_UART_Receive_DMA(self->huart, self->rx_dma, sizeof(self->rx_dma));
        return;
    }

    size_t write =
        sizeof(self->rx_dma) - __HAL_DMA_GET_COUNTER(self->huart->hdmarx);

    while (self->rx_read != write) {
        uint8_t byte = self->rx_dma[self->rx_read];
        self->rx_read = (self->rx_read + 1) % sizeof(self->rx_dma);

        PlrsImuFrame frame;
        if (plrs_imu_parser_feed(&self->parser, byte, HAL_GetTick(), &frame) ==
            PLRS_IMU_FRAME_OK) {
            PLRS_IMU__onFrame(self, &frame);
        }
    }
}

bool PLRS_IMU__getHeading(PLRS_IMU *self, float *deg_out) {
    if (!self->has_heading) {
        return false;
    }
    *deg_out = self->heading_deg;
    return true;
}

bool PLRS_IMU__isFresh(const PLRS_IMU *self, uint32_t timeout_ms) {
    if (!self->has_heading) {
        return false;
    }
    return HAL_GetTick() - self->heading_ms <= timeout_ms;
}

bool PLRS_IMU__getHeel(PLRS_IMU *self, float *deg_out) {
    if (!self->has_attitude) {
        return false;
    }
    *deg_out = self->heel_deg;
    return true;
}

bool PLRS_IMU__getYawRate(PLRS_IMU *self, float *dps_out) {
    if (!self->has_attitude) {
        return false;
    }
    *dps_out = self->yaw_rate_dps;
    return true;
}

bool PLRS_IMU__isAttitudeFresh(const PLRS_IMU *self, uint32_t timeout_ms) {
    if (!self->has_attitude) {
        return false;
    }
    return HAL_GetTick() - self->attitude_ms <= timeout_ms;
}

bool PLRS_IMU__isHeadingValid(const PLRS_IMU *self) {
    return self->heading_valid;
}

uint32_t PLRS_IMU__lastFrameTick(const PLRS_IMU *self) {
    return self->last_rx_ms;
}

uint32_t PLRS_IMU__drops(const PLRS_IMU *self) {
    return self->drops;
}
