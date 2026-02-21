/*
 * GPS.c
 *
 *  Created on: Jan 25, 2026
 *      Author: George Sleen
 */

#include "GPS.h"
#include "debug_log.h"
#include <stdlib.h>
#include <string.h>

/*
 * Constants
 */

static const uint32_t MESSAGE_GLL = 0x4C4C47;
static const uint32_t MESSAGE_VTG = 0x475456;
static const uint32_t MESSAGE_GGA = 0x414747;
static const uint32_t MESSAGE_RMC = 0x434D52;

static const uint8_t GLL_LATITUDE_INDEX = 1;
static const uint8_t GLL_LATITUDE_HEMISPHERE_INDEX = 2;
static const uint8_t GLL_LONGITUDE_INDEX = 3;
static const uint8_t GLL_LONGITUDE_HEMISPHERE_INDEX = 4;
static const uint8_t GLL_TIME_INDEX = 5;
static const uint8_t GLL_STATUS_INDEX = 6;

static const uint8_t GGA_TIME_INDEX = 1;
static const uint8_t GGA_LATITUDE_INDEX = 2;
static const uint8_t GGA_LATITUDE_HEMISPHERE_INDEX = 3;
static const uint8_t GGA_LONGITUDE_INDEX = 4;
static const uint8_t GGA_LONGITUDE_HEMISPHERE_INDEX = 5;
static const uint8_t GGA_FIX_QUALITY_INDEX = 6;

static const uint8_t RMC_TIME_INDEX = 1;
static const uint8_t RMC_STATUS_INDEX = 2;
static const uint8_t RMC_LATITUDE_INDEX = 3;
static const uint8_t RMC_LATITUDE_HEMISPHERE_INDEX = 4;
static const uint8_t RMC_LONGITUDE_INDEX = 5;
static const uint8_t RMC_LONGITUDE_HEMISPHERE_INDEX = 6;
static const uint8_t RMC_SPEED_KNOTS_INDEX = 7;

static const uint8_t VTG_SPEED_KMH_INDEX = 7;

static const uint32_t GPS_FRAME_ID = 0x070U;
static const uint32_t GPS_FRAME_LENGTH = FDCAN_DLC_BYTES_20;

static const uint32_t GPS_DEGREES_SCALE = 1000000U;
static const uint32_t GPS_MINUTES_DIVISOR = 60U;
static const uint32_t GPS_LATITUDE_OFFSET = 90U;
static const uint32_t GPS_LONGITUDE_OFFSET = 180U;
static const uint32_t GPS_TIME_SCALE = 1000U;

/*
 * Helper functions
 */

static uint32_t parseDigits(const char *value, uint8_t max_digits,
                            uint8_t *digits_read) {
  uint32_t result = 0;
  uint8_t count = 0;
  while (value && *value && *value >= '0' && *value <= '9' &&
         count < max_digits) {
    result = (result * 10U) + (uint32_t)(*value - '0');
    value++;
    count++;
  }
  if (digits_read) {
    *digits_read = count;
  }
  return result;
}

static bool parseTime(const char *value, uint32_t *seconds_ms, uint8_t *minutes,
                      uint8_t *hours) {
  if (!value || !seconds_ms || !minutes || !hours) {
    return false;
  }

  uint8_t digits_read = 0;
  uint32_t hhmmss = parseDigits(value, 6, &digits_read);
  if (digits_read < 6) {
    return false;
  }

  uint8_t hour_value = (uint8_t)(hhmmss / 10000U);
  uint8_t minute_value = (uint8_t)((hhmmss / 100U) % 100U);
  uint8_t second_value = (uint8_t)(hhmmss % 100U);

  uint32_t fractional_ms = 0U;
  const char *dot = strchr(value, '.');
  if (dot != NULL) {
    uint8_t frac_digits = 0;
    uint32_t frac_value = parseDigits(dot + 1, 3, &frac_digits);
    if (frac_digits == 1) {
      fractional_ms = frac_value * 100U;
    } else if (frac_digits == 2) {
      fractional_ms = frac_value * 10U;
    } else if (frac_digits >= 3) {
      fractional_ms = frac_value;
    }
  }

  *hours = hour_value;
  *minutes = minute_value;
  *seconds_ms = ((uint32_t)second_value * GPS_TIME_SCALE) + fractional_ms;
  return true;
}

static bool parseLatitudeLongitude(const char *latitude_value,
                                   const char *latitude_hemisphere,
                                   const char *longitude_value,
                                   const char *longitude_hemisphere,
                                   uint32_t *latitude_scaled,
                                   uint32_t *longitude_scaled) {
  if (!latitude_value || !longitude_value || !latitude_hemisphere ||
      !longitude_hemisphere || !latitude_scaled || !longitude_scaled) {
    return false;
  }

  size_t latitude_len = strlen(latitude_value);
  size_t longitude_len = strlen(longitude_value);
  if (latitude_len < 4U || longitude_len < 5U) {
    return false;
  }

  uint8_t latitude_digits = 0;
  uint8_t longitude_digits = 0;
  uint32_t latitude_integer = parseDigits(latitude_value, 10, &latitude_digits);
  uint32_t longitude_integer =
      parseDigits(longitude_value, 10, &longitude_digits);

  if (latitude_digits < 4U || longitude_digits < 5U) {
    return false;
  }

  uint32_t latitude_degrees = latitude_integer / 100U;
  uint32_t longitude_degrees = longitude_integer / 100U;

  double latitude_raw = strtod(latitude_value, NULL);
  double longitude_raw = strtod(longitude_value, NULL);

  double latitude_minutes = latitude_raw - (double)(latitude_degrees * 100U);
  double longitude_minutes =
      longitude_raw - (double)(longitude_degrees * 100U);

  if (latitude_minutes < 0.0 || latitude_minutes >= 60.0 ||
      longitude_minutes < 0.0 || longitude_minutes >= 60.0) {
    return false;
  }

  int64_t latitude_decimal_scaled =
      ((int64_t)latitude_degrees * GPS_DEGREES_SCALE) +
      (int64_t)(((latitude_minutes * (double)GPS_DEGREES_SCALE) /
                 (double)GPS_MINUTES_DIVISOR) +
                0.5);
  int64_t longitude_decimal_scaled =
      ((int64_t)longitude_degrees * GPS_DEGREES_SCALE) +
      (int64_t)(((longitude_minutes * (double)GPS_DEGREES_SCALE) /
                 (double)GPS_MINUTES_DIVISOR) +
                0.5);

  if (latitude_hemisphere[0] == 'S') {
    latitude_decimal_scaled = -latitude_decimal_scaled;
  } else if (latitude_hemisphere[0] != 'N') {
    return false;
  }

  if (longitude_hemisphere[0] == 'W') {
    longitude_decimal_scaled = -longitude_decimal_scaled;
  } else if (longitude_hemisphere[0] != 'E') {
    return false;
  }

  latitude_decimal_scaled += (int64_t)GPS_LATITUDE_OFFSET * GPS_DEGREES_SCALE;
  longitude_decimal_scaled += (int64_t)GPS_LONGITUDE_OFFSET * GPS_DEGREES_SCALE;

  if (latitude_decimal_scaled < 0 || longitude_decimal_scaled < 0) {
    return false;
  }

  *latitude_scaled = (uint32_t)latitude_decimal_scaled;
  *longitude_scaled = (uint32_t)longitude_decimal_scaled;
  return true;
}

static bool parseSpeedKmh(const char *value, uint32_t *speed_thousandths) {
  if (!value || !speed_thousandths) {
    return false;
  }

  uint32_t integer_part = 0U;
  uint32_t fraction_part = 0U;
  uint8_t fraction_digits = 0U;

  const char *dot = strchr(value, '.');
  if (dot != NULL) {
    integer_part = parseDigits(value, 10, NULL);
    fraction_part = parseDigits(dot + 1, 3, &fraction_digits);
  } else {
    integer_part = parseDigits(value, 10, NULL);
  }

  if (fraction_digits == 1U) {
    fraction_part *= 100U;
  } else if (fraction_digits == 2U) {
    fraction_part *= 10U;
  }

  *speed_thousandths = (integer_part * 1000U) + fraction_part;
  return true;
}

static bool parseSpeedKnotsToKmh(const char *value,
                                 uint32_t *speed_kmh_thousandths) {
  uint32_t knots_thousandths = 0U;
  if (!parseSpeedKmh(value, &knots_thousandths)) {
    return false;
  }
  // Convert knots to km/h with rounding: km/h = knots * 1.852
  uint64_t scaled = (uint64_t)knots_thousandths * 1852ULL;
  *speed_kmh_thousandths = (uint32_t)((scaled + 500ULL) / 1000ULL);
  return true;
}
/*
 * Management
 */

GPS *GPS__create(NMEA0183 *nmea_channel) {
  if (!nmea_channel) {
    return NULL;
  }

  GPS *self = (GPS *)malloc(sizeof(GPS));
  if (!self) {
    return NULL;
  }

  memset(self, 0, sizeof(GPS));
  self->channel = nmea_channel;
  return self;
}

void GPS__destroy(GPS *self) {
  if (self) {
    free(self);
  }
}

bool GPS__poll(GPS *self) {
  if (!self || !self->channel) {
    return false;
  }

  NMEA0183Raw *message = NMEA0183__getTopBufferItem(self->channel);
  if (!message) {
    return false;
  }

  bool parsed = GPS__parseMessage(self, message);
  NMEA0183__incrementReadIndex(self->channel);
  return parsed;
}

bool GPS__parseMessage(GPS *self, NMEA0183Raw *message) {
  if (!self || !message) {
    return false;
  }

  if (NMEA0183__checkMessage(message) != GOOD_MESSAGE) {
    return false;
  }

  uint32_t sentence_type = NMEA0183__getScentenceType(message);
  bool parsed = false;

  if (sentence_type == MESSAGE_GLL) {
    const char *latitude =
        (const char *)NMEA0183__getField(message, GLL_LATITUDE_INDEX);
    const char *latitude_hemisphere = (const char *)NMEA0183__getField(
        message, GLL_LATITUDE_HEMISPHERE_INDEX);
    const char *longitude =
        (const char *)NMEA0183__getField(message, GLL_LONGITUDE_INDEX);
    const char *longitude_hemisphere = (const char *)NMEA0183__getField(
        message, GLL_LONGITUDE_HEMISPHERE_INDEX);
    const char *time_str =
        (const char *)NMEA0183__getField(message, GLL_TIME_INDEX);
    const char *status =
        (const char *)NMEA0183__getField(message, GLL_STATUS_INDEX);

    if (status && status[0] == 'A') {
      uint32_t latitude_scaled = 0U;
      uint32_t longitude_scaled = 0U;
      uint32_t seconds_ms = 0U;
      uint8_t minutes = 0U;
      uint8_t hours = 0U;

      if (parseLatitudeLongitude(latitude, latitude_hemisphere, longitude,
                                 longitude_hemisphere, &latitude_scaled,
                                 &longitude_scaled) &&
          parseTime(time_str, &seconds_ms, &minutes, &hours)) {
        self->latitude = latitude_scaled;
        self->longitude = longitude_scaled;
        self->utc_seconds_ms = seconds_ms;
        self->utc_minutes = minutes;
        self->utc_hours = hours;
        self->position_valid = true;
        self->time_valid = true;
        parsed = true;
        DEBUG_PRINTF("[GPS] GLL parsed lat=%lu lon=%lu t=%02u:%02u.%03lu\r\n",
                     (unsigned long)self->latitude,
                     (unsigned long)self->longitude, (unsigned)self->utc_hours,
                     (unsigned)self->utc_minutes,
                     (unsigned long)self->utc_seconds_ms);
      } else {
        self->position_valid = false;
        self->time_valid = false;
      }
    } else {
      self->position_valid = false;
      self->time_valid = false;
    }
  } else if (sentence_type == MESSAGE_GGA) {
    const char *time_str =
        (const char *)NMEA0183__getField(message, GGA_TIME_INDEX);
    const char *latitude =
        (const char *)NMEA0183__getField(message, GGA_LATITUDE_INDEX);
    const char *latitude_hemisphere = (const char *)NMEA0183__getField(
        message, GGA_LATITUDE_HEMISPHERE_INDEX);
    const char *longitude =
        (const char *)NMEA0183__getField(message, GGA_LONGITUDE_INDEX);
    const char *longitude_hemisphere = (const char *)NMEA0183__getField(
        message, GGA_LONGITUDE_HEMISPHERE_INDEX);
    const char *fix_quality =
        (const char *)NMEA0183__getField(message, GGA_FIX_QUALITY_INDEX);

    if (fix_quality && fix_quality[0] != '0') {
      uint32_t latitude_scaled = 0U;
      uint32_t longitude_scaled = 0U;
      uint32_t seconds_ms = 0U;
      uint8_t minutes = 0U;
      uint8_t hours = 0U;

      if (parseLatitudeLongitude(latitude, latitude_hemisphere, longitude,
                                 longitude_hemisphere, &latitude_scaled,
                                 &longitude_scaled) &&
          parseTime(time_str, &seconds_ms, &minutes, &hours)) {
        self->latitude = latitude_scaled;
        self->longitude = longitude_scaled;
        self->utc_seconds_ms = seconds_ms;
        self->utc_minutes = minutes;
        self->utc_hours = hours;
        self->position_valid = true;
        self->time_valid = true;
        parsed = true;
        DEBUG_PRINTF("[GPS] GGA parsed lat=%lu lon=%lu t=%02u:%02u.%03lu\r\n",
                     (unsigned long)self->latitude,
                     (unsigned long)self->longitude, (unsigned)self->utc_hours,
                     (unsigned)self->utc_minutes,
                     (unsigned long)self->utc_seconds_ms);
      } else {
        self->position_valid = false;
        self->time_valid = false;
      }
    } else {
      self->position_valid = false;
      self->time_valid = false;
    }
  } else if (sentence_type == MESSAGE_RMC) {
    const char *time_str =
        (const char *)NMEA0183__getField(message, RMC_TIME_INDEX);
    const char *status =
        (const char *)NMEA0183__getField(message, RMC_STATUS_INDEX);
    const char *latitude =
        (const char *)NMEA0183__getField(message, RMC_LATITUDE_INDEX);
    const char *latitude_hemisphere = (const char *)NMEA0183__getField(
        message, RMC_LATITUDE_HEMISPHERE_INDEX);
    const char *longitude =
        (const char *)NMEA0183__getField(message, RMC_LONGITUDE_INDEX);
    const char *longitude_hemisphere = (const char *)NMEA0183__getField(
        message, RMC_LONGITUDE_HEMISPHERE_INDEX);
    const char *speed_knots =
        (const char *)NMEA0183__getField(message, RMC_SPEED_KNOTS_INDEX);

    if (status && status[0] == 'A') {
      uint32_t latitude_scaled = 0U;
      uint32_t longitude_scaled = 0U;
      uint32_t seconds_ms = 0U;
      uint8_t minutes = 0U;
      uint8_t hours = 0U;
      uint32_t speed_kmh_thousandths = 0U;

      bool ok = parseLatitudeLongitude(latitude, latitude_hemisphere, longitude,
                                       longitude_hemisphere, &latitude_scaled,
                                       &longitude_scaled) &&
                parseTime(time_str, &seconds_ms, &minutes, &hours);
      if (ok && parseSpeedKnotsToKmh(speed_knots, &speed_kmh_thousandths)) {
        self->latitude = latitude_scaled;
        self->longitude = longitude_scaled;
        self->utc_seconds_ms = seconds_ms;
        self->utc_minutes = minutes;
        self->utc_hours = hours;
        self->speed_kmh_thousandths = speed_kmh_thousandths;
        self->position_valid = true;
        self->time_valid = true;
        self->speed_valid = true;
        parsed = true;
        DEBUG_PRINTF("[GPS] RMC parsed lat=%lu lon=%lu spd=%lu t=%02u:%02u.%03lu\r\n",
                     (unsigned long)self->latitude,
                     (unsigned long)self->longitude,
                     (unsigned long)self->speed_kmh_thousandths,
                     (unsigned)self->utc_hours, (unsigned)self->utc_minutes,
                     (unsigned long)self->utc_seconds_ms);
      } else {
        self->position_valid = false;
        self->time_valid = false;
        self->speed_valid = false;
      }
    } else {
      self->position_valid = false;
      self->time_valid = false;
      self->speed_valid = false;
    }
  } else if (sentence_type == MESSAGE_VTG) {
    const char *speed_kmh =
        (const char *)NMEA0183__getField(message, VTG_SPEED_KMH_INDEX);
    uint32_t speed_thousandths = 0U;
    if (parseSpeedKmh(speed_kmh, &speed_thousandths)) {
      self->speed_kmh_thousandths = speed_thousandths;
      self->speed_valid = true;
      parsed = true;
      DEBUG_PRINTF("[GPS] VTG parsed spd=%lu\r\n",
                   (unsigned long)self->speed_kmh_thousandths);
    } else {
      self->speed_valid = false;
    }
  }

  return parsed;
}
HAL_StatusTypeDef GPS__CAN_transmit(const GPS *self,
                                    FDCAN_HandleTypeDef *hfdcan1) {
  if (!self || !hfdcan1) {
    return HAL_ERROR;
  }

  if (!self->position_valid || !self->time_valid || !self->speed_valid) {
    return HAL_ERROR;
  }

  uint8_t payload[20];
  memset(payload, 0, sizeof(payload));

  payload[0] = (uint8_t)(self->latitude & 0xFF);
  payload[1] = (uint8_t)((self->latitude >> 8) & 0xFF);
  payload[2] = (uint8_t)((self->latitude >> 16) & 0xFF);
  payload[3] = (uint8_t)((self->latitude >> 24) & 0xFF);

  payload[4] = (uint8_t)(self->longitude & 0xFF);
  payload[5] = (uint8_t)((self->longitude >> 8) & 0xFF);
  payload[6] = (uint8_t)((self->longitude >> 16) & 0xFF);
  payload[7] = (uint8_t)((self->longitude >> 24) & 0xFF);

  payload[8] = (uint8_t)(self->utc_seconds_ms & 0xFF);
  payload[9] = (uint8_t)((self->utc_seconds_ms >> 8) & 0xFF);
  payload[10] = (uint8_t)((self->utc_seconds_ms >> 16) & 0xFF);
  payload[11] = (uint8_t)((self->utc_seconds_ms >> 24) & 0xFF);

  payload[12] = self->utc_minutes;
  payload[13] = self->utc_hours;

  payload[16] = (uint8_t)(self->speed_kmh_thousandths & 0xFF);
  payload[17] = (uint8_t)((self->speed_kmh_thousandths >> 8) & 0xFF);
  payload[18] = (uint8_t)((self->speed_kmh_thousandths >> 16) & 0xFF);
  payload[19] = (uint8_t)((self->speed_kmh_thousandths >> 24) & 0xFF);

  DEBUG_PRINTF("[GPS] CAN tx id=0x%03lX lat=%lu lon=%lu spd=%lu\r\n",
               (unsigned long)GPS_FRAME_ID, (unsigned long)self->latitude,
               (unsigned long)self->longitude,
               (unsigned long)self->speed_kmh_thousandths);
  return CAN_Transmit(GPS_FRAME_ID, FDCAN_STANDARD_ID, GPS_FRAME_LENGTH,
                      payload, hfdcan1);
}
