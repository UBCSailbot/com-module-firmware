/*
 * WIND_SENSOR.c
 *
 *  Created on: Nov. 15, 2025
 *      Author: george-sleen
 */

#include <NMEA0183.h>
#include <WIND_SENSOR.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Constants
 */

static const uint8_t MESSAGE_TYPE_INDEX = 0;

// Wind temperature messages
static const char WIND_TEMP_SENTENCE[] = "WIXDR";
static const uint8_t WIND_TEMP_INDEX = 2;

// Wind messages
static const char WIND_SENTENCE[] = "IIMWV";
static const uint8_t WIND_DIRECTION_INDEX = 1;
static const uint8_t WIND_REFERENCE_INDEX = 2;
static const uint8_t WIND_SPEED_INDEX = 3;
static const uint8_t WIND_STATUS_INDEX = 5;

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

/**
 * Print a fixed-point value scaled by 10 with a label.
 *
 * @param label the output label
 * @param value the fixed-point value
 * @return void
 */
static void printTenths(const char *label, tenths16_t value) {
  printf("%s=%u.%u", label, (unsigned int)(value / 10),
         (unsigned int)(value % 10));
}

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
  bool result = false;

  if (!self) {
    return false;
  }

  NMEA0183 *channel = self->channel;
  NMEA0183Raw *message = NMEA0183__getTopBufferItem(channel);

  if (message != NULL && NMEA0183__checkMessage(message) == GOOD_MESSAGE) {
    const char *messageType = getField(message, MESSAGE_TYPE_INDEX);

    if (messageType && strcmp(messageType, WIND_SENTENCE) == 0) {
      const char *direction = getField(message, WIND_DIRECTION_INDEX);
      const char *reference = getField(message, WIND_REFERENCE_INDEX);
      const char *speed = getField(message, WIND_SPEED_INDEX);
      const char *status = getField(message, WIND_STATUS_INDEX);

      self->direction = parseTenths(direction);
      self->speed = parseTenths(speed);
      self->reference =
          reference ? (wind_reference_t)reference[0] : (wind_reference_t)0;
      self->status = status ? (wind_status_t)status[0] : UNKNOWN;

      result = true;
    } else if (messageType && strcmp(messageType, WIND_TEMP_SENTENCE) == 0) {
      const char *temp = getField(message, WIND_TEMP_INDEX);

      self->temp = parseTenths(temp);

    } else {
      result = false;
    }
  }

  // Advance ring buffer
  NMEA0183__incrementReadIndex(channel);
  return result;
}

/*
 * Prints the current wind sensor values to stdout.
 */
void WIND_SENSOR__print(const WIND_SENSOR *self) {
  if (!self) {
    return;
  }

  printTenths("dir", self->direction);
  printf(" %c ", referenceToChar(self->reference));
  printTenths("spd", self->speed);
  printf(" kt status=%c ", statusToChar(self->status));
  printTenths("temp", self->temp);
  printf(" C\r\n");
}
