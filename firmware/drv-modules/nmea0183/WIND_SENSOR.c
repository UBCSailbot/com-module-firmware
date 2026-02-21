/*
 * WIND_SENSOR.c
 *
 *  Created on: Nov. 15, 2025
 *      Author: george-sleen
 */

#include "can.h"
#include "stm32u5xx_hal_def.h"
#include "stm32u5xx_hal_fdcan.h"
#include "debug_log.h"
#include <NMEA0183.h>
#include <WIND_SENSOR.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Constants
 */

// Temperature messages
static const uint8_t WIND_TEMP_INDEX = 2;

// Wind data messages
static const uint8_t WIND_DIRECTION_INDEX = 1;
static const uint8_t WIND_REFERENCE_INDEX = 2;
static const uint8_t WIND_SPEED_INDEX = 3;
static const uint8_t WIND_STATUS_INDEX = 5;

// CAN communication
static const can_frame_id_t SAIL_WIND_ID = 0x040;
static const can_frame_id_t DATA_WIND_ID = 0x041;
static const uint32_t WIND_DATA_LENGTH = FDCAN_DLC_BYTES_4;

/*
 * Helper functions
 */

/**
 * Convert a string to an integer by ignoring non-digit characters.
 *
 * @param str the string to convert
 * @return the corresponding integer
 */
static int stringToInt(const char *str) {
  int value = 0;

  if (!str) {
    return 0;
  }

  while (*str != '\0') {
    if (*str >= '0' && *str <= '9') {
      value = (value * 10) + (*str - '0');
    }
    str++;
  }

  return value;
}

/**
 * Return a field pointer as a C string.
 *
 * @param message the NMEA message to read
 * @param index the field index to fetch
 * @return pointer to the field or NULL
 */
static const char *getField(NMEA0183Raw *message, uint8_t index) {
  return (const char *)NMEA0183__getField(message, index);
}

/**
 * Parse a fixed-point value with one decimal place.
 *
 * @param value_str the input field string
 * @return fixed-point value scaled by 10
 */
static tenths16_t parseTenths(const char *value_str) {
  return (tenths16_t)stringToInt(value_str);
}

#if APP_DEBUG_LOG
/**
 * Convert a wind status to a printable character.
 *
 * @param status the parsed status enum
 * @return printable character for the status
 */
static char statusToChar(wind_status_t status) {
  if (status == VALID || status == INVALID) {
    return (char)status;
  }
  return '?';
}

/**
 * Convert a wind reference to a printable character.
 *
 * @param reference the parsed reference enum
 * @return printable character for the reference
 */
static char referenceToChar(wind_reference_t reference) {
  if (reference == REFERENCE) {
    return (char)reference;
  }
  return '?';
}
#endif

#if APP_DEBUG_LOG
/**
 * Print a fixed-point value scaled by 10 with a label.
 *
 * @param label the output label
 * @param value the fixed-point value
 * @return void
 */
static void printTenths(const char *label, tenths16_t value) {
  DEBUG_PRINTF("%s=%u.%u", label, (unsigned int)(value / 10),
               (unsigned int)(value % 10));
}
#endif

/*
 * Management
 */

/**
 * Creates a new WIND_SENSOR object.
 *
 * @param huartChannel Is the USART channel associated with this object. The
 * object must not be mutated after calling this function.
 * @return An initialized WIND_SENSOR object.
 */
WIND_SENSOR *WIND_SENSOR__create(NMEA0183 *nmeaChannel) {
  if (!nmeaChannel) {
    return NULL;
  }

  WIND_SENSOR *self = (WIND_SENSOR *)malloc(sizeof(WIND_SENSOR));
  if (!self) {
    return NULL;
  }

  memset(self, 0, sizeof(WIND_SENSOR));

  self->channel = nmeaChannel;
  self->reference = REFERENCE;
  self->status = UNKNOWN;

  return self;
}

/**
 * Deletes the WIND_SENSOR object.
 *
 * @param self Must be an initialized WIND_SENSOR object
 */
void WIND_SENSOR__destroy(WIND_SENSOR *self) {
  if (self) {
    free(self);
  }
}

/**
 * Polls the NMEA channel and updates the WIND_SENSOR object with the most
 * recent data.
 *
 * @param self must be an initialized WIND_SENSOR object (probably connected to
 * a wind sensor if you want it to do anything useful...)
 */
bool WIND_SENSOR__poll(WIND_SENSOR *self) {
  if (!self || !self->channel) {
    return false;
  }

  NMEA0183Raw *message = NMEA0183__getTopBufferItem(self->channel);
  if (!message) {
    return false;
  }

  bool result = WIND_SENSOR__parseMessage(self, message);
  NMEA0183__incrementReadIndex(self->channel);
  return result;
}

bool WIND_SENSOR__parseMessage(WIND_SENSOR *self, NMEA0183Raw *message) {
  if (!self || !message) {
    return false;
  }

  if (NMEA0183__checkMessage(message) != GOOD_MESSAGE) {
    return false;
  }

  uint32_t sentenceType = NMEA0183__getScentenceType(message);

  if (sentenceType == MESSAGE_MWV) {
    const char *direction = getField(message, WIND_DIRECTION_INDEX);
    const char *reference = getField(message, WIND_REFERENCE_INDEX);
    const char *speed = getField(message, WIND_SPEED_INDEX);
    const char *status = getField(message, WIND_STATUS_INDEX);

    self->direction = parseTenths(direction);
    self->speed = parseTenths(speed);
    self->reference =
        reference ? (wind_reference_t)reference[0] : (wind_reference_t)0;
    self->status = status ? (wind_status_t)status[0] : UNKNOWN;
    DEBUG_PRINTF("[WIND] MWV parsed dir_tenths=%u spd_tenths=%u ref=%c status=%c\r\n",
                 (unsigned)self->direction, (unsigned)self->speed,
                 referenceToChar(self->reference), statusToChar(self->status));

    return true;
  }

  if (sentenceType == MESSAGE_XDR) {
    const char *temp = getField(message, WIND_TEMP_INDEX);

    self->temp = parseTenths(temp);
    DEBUG_PRINTF("[WIND] XDR parsed temp_tenths=%u\r\n", (unsigned)self->temp);
    return true;
  }

  return false;
}

/**
 * Prints the current wind sensor values to stdout.
 */
void WIND_SENSOR__print(const WIND_SENSOR *self) {
  if (!self) {
    return;
  }

#if APP_DEBUG_LOG
  printTenths("dir", self->direction);
  DEBUG_PRINTF(" %c ", referenceToChar(self->reference));
  printTenths("spd", self->speed);
  DEBUG_PRINTF(" kt status=%c ", statusToChar(self->status));
  printTenths("temp", self->temp);
  DEBUG_PRINTF(" C\r\n");
#else
  (void)self;
#endif
}

/**
 *  Transmit SAIL_WIND or DATA_WIND over CANFD
 */
static HAL_StatusTypeDef
WIND_SENSOR__CAN_transmit_single(WIND_SENSOR *self, can_frame_id_t CAN_ID,
                                 FDCAN_HandleTypeDef *hfdcan1) {
  uint8_t data[4];
  uint16_t angle_deg = (uint16_t)(self->direction / 10U);
  uint16_t speed_tenths = (uint16_t)(self->speed);

  // Pack angle into [15:0]
  data[0] = (uint8_t)(angle_deg & 0xFF);
  data[1] = (uint8_t)((angle_deg >> 8) & 0xFF);
  // Pack speed into [31:16]
  data[2] = (uint8_t)(speed_tenths & 0xFF);
  data[3] = (uint8_t)((speed_tenths >> 8) & 0xFF);

  DEBUG_PRINTF("[WIND] CAN tx id=0x%03lX angle=%u speed_tenths=%u\r\n",
               (unsigned long)CAN_ID, (unsigned)angle_deg,
               (unsigned)speed_tenths);
  return CAN_Transmit((uint32_t)CAN_ID, FDCAN_STANDARD_ID, WIND_DATA_LENGTH,
                      data, hfdcan1);
}

/**
 *  Transmit both SAIL_WIND and DATA_WIND over CANFD
 *  as defined in [Sailbot's Confluence Page]
 *  (https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1827176527/CAN+Frames)
 *
 *  @param self an initialized WIND_SENSOR object.
 *  @param hfdcan1 a can channel.
 *  @return HAL_OK if successful, a HAL error code otherwise.
 */
HAL_StatusTypeDef WIND_SENSOR__CAN_transmit(WIND_SENSOR *self,
                                            FDCAN_HandleTypeDef *hfdcan1) {
  HAL_StatusTypeDef sailTransmitted =
      WIND_SENSOR__CAN_transmit_single(self, SAIL_WIND_ID, hfdcan1);
  HAL_StatusTypeDef dataTransmitted =
      WIND_SENSOR__CAN_transmit_single(self, DATA_WIND_ID, hfdcan1);

  if (sailTransmitted != HAL_OK)
    return sailTransmitted;
  return dataTransmitted;
}
