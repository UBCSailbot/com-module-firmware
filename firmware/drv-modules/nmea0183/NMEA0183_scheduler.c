/*
 * NMEA0183_scheduler.c
 *
 *  Created on: Jan 25, 2026
 *      Author: George Sleen
 */

#include "NMEA0183_scheduler.h"
#include "debug_log.h"
#include "main.h"

/*
 * Constants
 */

static const uint32_t MESSAGE_GLL = 0x4C4C47;
static const uint32_t MESSAGE_VTG = 0x475456;
static const uint32_t MESSAGE_GGA = 0x414747;
static const uint32_t MESSAGE_RMC = 0x434D52;
static const uint32_t CAN_SEND_INTERVAL_MS = 500U;

static void NMEA0183__toggle_can_led(void) {
#if defined(LED_RED_GPIO_Port) && defined(LED_RED_Pin)
  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
#endif
}

bool NMEA0183__scheduler_step(NMEA0183_Scheduler *scheduler, uint32_t now_ms) {
  if (!scheduler || !scheduler->channel) {
    return false;
  }

  bool parsed = false;
  NMEA0183Raw *message = NMEA0183__getTopBufferItem(scheduler->channel);
  if (message) {
    uint32_t sentence_type = NMEA0183__getScentenceType(message);
    DEBUG_PRINTF("[NMEA] sentence=0x%06lX\r\n", (unsigned long)sentence_type);

    if (sentence_type == MESSAGE_VDM || sentence_type == MESSAGE_VDO) {
      if (NMEA0183__checkMessage(message) != GOOD_MESSAGE) {
        DEBUG_PRINTF("[AIS] invalid NMEA message\r\n");
        NMEA0183__incrementReadIndex(scheduler->channel);
      } else if (scheduler->ais_parser && scheduler->ais_batch &&
                 scheduler->hfdcan1) {
        AIS_DATA *data = AIS__parseNMEAMessage(scheduler->ais_parser, message);
        if (data) {
          DEBUG_PRINTF("AIS parsed\r\n");
          uint32_t prev_next_send_ms = scheduler->ais_batch->next_send_ms;
          HAL_StatusTypeDef status = AIS__CAN_process(
              scheduler->ais_batch, data, now_ms, scheduler->hfdcan1);
          if (status == HAL_OK) {
            DEBUG_PRINTF("CAN TX AIS batch\r\n");
          }
          if ((status == HAL_OK) && (prev_next_send_ms != 0U) &&
              ((int32_t)(now_ms - prev_next_send_ms) >= 0)) {
            scheduler->last_ais_send_ms = now_ms;
            NMEA0183__toggle_can_led();
          }
          DEBUG_PRINTF("[AIS] parsed mmsi=%lu status=%d ships=%u\r\n",
                       (unsigned long)AIS__getMMSINumber(data), (int)status,
                       (unsigned)scheduler->ais_batch->ship_count);
          parsed = true;
        } else {
          DEBUG_PRINTF("[AIS] parser awaiting multipart/unsupported payload\r\n");
        }
        NMEA0183__incrementReadIndex(scheduler->channel);
      } else {
        DEBUG_PRINTF("[AIS] scheduler missing parser or CAN handle\r\n");
        NMEA0183__incrementReadIndex(scheduler->channel);
      }
    } else if (sentence_type == MESSAGE_MWV || sentence_type == MESSAGE_XDR) {
      if (scheduler->wind_sensor && scheduler->hfdcan1) {
        parsed = WIND_SENSOR__parseMessage(scheduler->wind_sensor, message);
        if (parsed) {
          HAL_StatusTypeDef status = WIND_SENSOR__CAN_transmit(
              scheduler->wind_sensor, scheduler->hfdcan1);
          DEBUG_PRINTF("[WIND] parsed+tx status=%d dir_tenths=%u spd_tenths=%u temp_tenths=%u\r\n",
                       (int)status, (unsigned)scheduler->wind_sensor->direction,
                       (unsigned)scheduler->wind_sensor->speed,
                       (unsigned)scheduler->wind_sensor->temp);
          if (status == HAL_OK) {
            NMEA0183__toggle_can_led();
          }
        } else {
          DEBUG_PRINTF("[WIND] message parse failed\r\n");
        }
      }
      NMEA0183__incrementReadIndex(scheduler->channel);
    } else if (sentence_type == MESSAGE_GLL || sentence_type == MESSAGE_VTG ||
               sentence_type == MESSAGE_GGA || sentence_type == MESSAGE_RMC) {
      if (scheduler->gps && scheduler->hfdcan1) {
        parsed = GPS__parseMessage(scheduler->gps, message);
        if (parsed) {
          DEBUG_PRINTF("GPS parsed\r\n");
          HAL_StatusTypeDef status =
              GPS__CAN_transmit(scheduler->gps, scheduler->hfdcan1);
          if (status == HAL_OK) {
            DEBUG_PRINTF("CAN TX GPS\r\n");
          }
          DEBUG_PRINTF("[GPS] parsed+tx status=%d lat=%lu lon=%lu speed=%lu\r\n",
                       (int)status, (unsigned long)scheduler->gps->latitude,
                       (unsigned long)scheduler->gps->longitude,
                       (unsigned long)scheduler->gps->speed_kmh_thousandths);
          if (status == HAL_OK) {
            scheduler->last_gps_send_ms = now_ms;
            NMEA0183__toggle_can_led();
          }
        } else {
          DEBUG_PRINTF("[GPS] message parse failed\r\n");
        }
      }
      NMEA0183__incrementReadIndex(scheduler->channel);
    } else {
      DEBUG_PRINTF("[NMEA] unsupported sentence type\r\n");
      NMEA0183__incrementReadIndex(scheduler->channel);
    }
  }

  // Periodic CAN sends at 2 Hz.
  if (scheduler->gps && scheduler->hfdcan1 && scheduler->gps->position_valid &&
      scheduler->gps->time_valid && scheduler->gps->speed_valid &&
      (now_ms - scheduler->last_gps_send_ms) >= CAN_SEND_INTERVAL_MS) {
    HAL_StatusTypeDef status =
        GPS__CAN_transmit(scheduler->gps, scheduler->hfdcan1);
    if (status == HAL_OK) {
      DEBUG_PRINTF("CAN TX GPS\r\n");
    }
    DEBUG_PRINTF("[GPS] periodic tx status=%d\r\n", (int)status);
    if (status == HAL_OK) {
      scheduler->last_gps_send_ms = now_ms;
      NMEA0183__toggle_can_led();
    }
  }

  if (scheduler->ais_batch && scheduler->hfdcan1 &&
      (now_ms - scheduler->last_ais_send_ms) >= CAN_SEND_INTERVAL_MS) {
    if (scheduler->ais_batch->ship_count == 0U) {
      HAL_StatusTypeDef status = AIS__CAN_transmit_empty(scheduler->hfdcan1);
      if (status == HAL_OK) {
        DEBUG_PRINTF("CAN TX AIS empty\r\n");
      }
      DEBUG_PRINTF("[AIS] periodic empty tx status=%d\r\n", (int)status);
      if (status == HAL_OK) {
        scheduler->last_ais_send_ms = now_ms;
        NMEA0183__toggle_can_led();
      }
    }
  }

  return parsed;
}
