/*
 * AIS.c
 *
 *  Created on: Apr 21, 2024
 *      Author: Michael Greenough
 */

#include "AIS.h"
#include "can.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Constants
 */

static const uint32_t AIS_FRAME_ID = 0x060;
static const uint32_t AIS_FRAME_LENGTH = FDCAN_DLC_BYTES_32;
static const uint32_t AIS_LATITUDE_OFFSET = 90U;
static const uint32_t AIS_LONGITUDE_OFFSET = 180U;
static const uint32_t AIS_DEGREES_SCALE = 1000000U;
static const uint32_t AIS_AIS_SCALE = 600000U;
static const uint16_t AIS_SOG_UNAVAILABLE = 1023U;
static const uint16_t AIS_COG_UNAVAILABLE = 3600U;
static const uint16_t AIS_HEADING_UNAVAILABLE = 511U;
static const int8_t AIS_ROT_UNAVAILABLE = -128;
static const uint32_t AIS_32BIT_MAX = 0xFFFFFFFFU;
static const uint8_t AIS_MAX_SHIPS_PER_BATCH = 127U;
static const uint32_t AIS_CAN_BATCH_INTERVAL_MS = 500U;

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------
// PFP
//---------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Converts from AIS specified 6-bit character into regular ASCII character.
 *
 * @param input Any valid 6-bit AIS character.
 * @return The corresponding valid ASCII character.
 */
uint8_t convertAscii(uint8_t input);

/*
 * Retrieves the bit sequence from the data based on a specified interval. The
 * input data is AIS specified 6-bit character string.
 *
 * @param data[] An array of 6-bit AIS data.
 * @param startPosition The starting bit position of the bit sequence. Must be
 * between 0 and a valid bit in the sequence (inclusive.)
 * @param endPosition The ending bit position of the bit sequence. Must be
 * between startPosition and a valid bit in the sequence (inclusive.)
 * @return The concatenated bit sequence as a 32 bit word.
 */
uint32_t getBinaryBits(uint8_t data[], uint16_t startPosition,
                       uint16_t endPosition);

/*
 * Convert from a 6-bit string to a regular ASCII string.
 *
 * @param input[] An array of 6-bit AIS data.
 * @param output[] The output ASCII string terminated with \0. Must be of size
 * at least (startBit-endBit) / 2 + 1
 * @param startBit The starting bit of the data to take the string from.
 * @param endBit The ending bit of the data to take the string from.
 */
void getAsciiString(uint8_t input[], uint8_t output[], uint16_t startBit,
                    uint16_t endBit);

/*
 * Take the byte of raw AIS data and converts it to regular binary.
 *
 * @param input Is a valid AIS ASCII byte.
 * @return The corresponding binary value.
 */
uint8_t convertSixBit(uint8_t input);

/*
 * Resets the time stamp of the AIS dictionary used for multi-part sentences.
 * Resetting just refers to making the time stamp 49 days in the future.
 *
 * @param aisMultiData is AIS_MULTI_SENTENCE to reset the time stamp of.
 */
void resetTimeStamp(AIS_MULTI_SENTENCE *aisMultiData);

static uint32_t AIS__convertLatitude(int32_t ais_lat);
static uint32_t AIS__convertLongitude(int32_t ais_lon);
static int AIS__findShipIndex(const AIS_CAN_BATCH *batch, uint32_t mmsi);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------
// OBJECT MANAGEMENT
//---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void AIS__init(AIS_PARSER *self) {
  memset(self, 0, sizeof(AIS_PARSER));
  for (uint8_t i = 0; i < 10; i++) {
    resetTimeStamp(&self->multiSentenceHeap[i]);
  }
}

AIS_PARSER *AIS__create() {
  AIS_PARSER *result = (AIS_PARSER *)malloc(sizeof(AIS_PARSER));
  AIS__init(result);
  return result;
}

void AIS__reset(AIS_PARSER *self) { AIS__init(self); }

void AIS__destroy(AIS_PARSER *data) {
  if (data) {
    AIS__reset(data);
    free(data);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------
// HELPER FUNCTIONS
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void resetTimeStamp(AIS_MULTI_SENTENCE *aisMultiData) {
  aisMultiData->timeStamp = 0xFFFF0000 - MULTI_SENTENCE_TIME_WINDOW;
}

uint8_t convertSixBit(uint8_t input) {
  if (input <= 87) {
    return input - 48;
  } else if (input <= 119) {
    return input - 56;
  } else {
    return 255;
  }
}

uint8_t convertAscii(uint8_t input) {
  if (input < 40) {
    return input + 64;
  } else {
    return input;
  }
}

uint32_t getBinaryBits(uint8_t data[], uint16_t startPosition,
                       uint16_t endPosition) {
  uint8_t startByte = startPosition / 6;
  uint8_t endByte = endPosition / 6;
  uint16_t length = endPosition - startPosition;
  uint32_t output = 0;
  for (uint8_t i = startByte; i <= endByte; i++) {
    output <<= 6;
    output += convertSixBit(data[i]);
  }
  output >>= 5 - ((endPosition) % 6);
  output &= (1 << (length + 1)) - 1;
  return output;
}

void getAsciiString(uint8_t input[], uint8_t output[], uint16_t startBit,
                    uint16_t endBit) {
  for (uint16_t i = startBit; i < endBit; i += 6) {
    output[(i - startBit) / 6] = convertAscii(getBinaryBits(input, i, i + 5));
  }
}

static uint32_t AIS__convertLatitude(int32_t ais_lat) {
  if (ais_lat == INT32_MAX) {
    return AIS_32BIT_MAX;
  }
  int64_t scaled = ((int64_t)ais_lat * AIS_DEGREES_SCALE) / AIS_AIS_SCALE;
  scaled += (int64_t)AIS_LATITUDE_OFFSET * AIS_DEGREES_SCALE;
  if (scaled < 0) {
    return 0;
  }
  if (scaled > (int64_t)AIS_32BIT_MAX) {
    return AIS_32BIT_MAX;
  }
  return (uint32_t)scaled;
}

static uint32_t AIS__convertLongitude(int32_t ais_lon) {
  if (ais_lon == INT32_MAX) {
    return AIS_32BIT_MAX;
  }
  int64_t scaled = ((int64_t)ais_lon * AIS_DEGREES_SCALE) / AIS_AIS_SCALE;
  scaled += (int64_t)AIS_LONGITUDE_OFFSET * AIS_DEGREES_SCALE;
  if (scaled < 0) {
    return 0;
  }
  if (scaled > (int64_t)AIS_32BIT_MAX) {
    return AIS_32BIT_MAX;
  }
  return (uint32_t)scaled;
}

static int AIS__findShipIndex(const AIS_CAN_BATCH *batch, uint32_t mmsi) {
  if (!batch) {
    return -1;
  }

  for (uint16_t index = 0; index < batch->ship_count; index++) {
    if (AIS__getMMSINumber((AIS_DATA *)&batch->ships[index]) == mmsi) {
      return (int)index;
    }
  }

  return -1;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------
// DATA PARSING FUNCTIONS
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

AIS_DATA *AIS__parseNMEAMessage(AIS_PARSER *self, NMEA0183Raw *data) {
  if (!self || !data) {
    NMEA_DEBUG_PRINT("[AIS] parse skipped: null self/data\r\n");
    return NULL;
  }

  uint8_t *aisBinary = NMEA0183__getField(data, 5);

  uint32_t sentence_type = NMEA0183__getScentenceType(data);
  if ((sentence_type == MESSAGE_VDM || sentence_type == MESSAGE_VDO) &&
      aisBinary != NULL) {
    uint8_t totalSentenceSegments = NMEA0183__getField(data, 1)[0] - '0';
    uint8_t aisBinaryLength = strlen((char *)aisBinary);
    NMEA_DEBUG_PRINT(
        "[AIS] type=0x%06lX parts=%u part=%c seq=%c ch=%c payload_len=%u\r\n",
        (unsigned long)sentence_type, totalSentenceSegments,
        NMEA0183__getField(data, 2) ? NMEA0183__getField(data, 2)[0] : '?',
        NMEA0183__getField(data, 3) ? NMEA0183__getField(data, 3)[0] : '?',
        NMEA0183__getField(data, 4) ? NMEA0183__getField(data, 4)[0] : '?',
        aisBinaryLength);

    if (totalSentenceSegments == 1) {
      // If this is a single sentence message then just copy it and return it.
      self->singleSentenceData.dataLength = aisBinaryLength;
      memcpy(self->singleSentenceData.sixBitData, aisBinary,
             self->singleSentenceData.dataLength);
      NMEA_DEBUG_PRINT("[AIS] single-part ready len=%u msg_id=%u mmsi=%lu\r\n",
                       self->singleSentenceData.dataLength,
                       AIS__getMessageID(&self->singleSentenceData),
                       (unsigned long)AIS__getMMSINumber(
                           &self->singleSentenceData));
      return &self->singleSentenceData;
    } else {
      uint8_t sequentialMessageIdentifier =
          NMEA0183__getField(data, 3)[0] - '0';

      // Check that the sequential identifier is in range, or else we will
      // access unallocated memory
      if (sequentialMessageIdentifier > 9)
        NMEA_DEBUG_PRINT("[AIS] drop: invalid multi sequence id=%u\r\n",
                         sequentialMessageIdentifier);
      if (sequentialMessageIdentifier > 9)
        return NULL;

      AIS_MULTI_SENTENCE *dictData =
          &self->multiSentenceHeap[sequentialMessageIdentifier];
      uint8_t vhfChannel = NMEA0183__getField(data, 4)[0];
      uint8_t sentenceNumber = NMEA0183__getField(data, 2)[0] - '0';

      // If this is the first part of the message then update the dictionary
      // with it
      if (sentenceNumber == 1) {
        dictData->timeStamp = HAL_GetTick();
        dictData->totalSentenceParts = totalSentenceSegments;
        dictData->lastSentenceIndex = 0;
        dictData->aisData.dataLength = 0;
        dictData->vhfChannel = vhfChannel;
        NMEA_DEBUG_PRINT("[AIS] multi start seq=%u parts=%u channel=%c\r\n",
                         sequentialMessageIdentifier, totalSentenceSegments,
                         vhfChannel);
      }

      // Make sure the dictionary element matches the incoming message
      if (dictData->vhfChannel != vhfChannel ||
          ++dictData->lastSentenceIndex != sentenceNumber ||
          dictData->totalSentenceParts != totalSentenceSegments ||
          HAL_GetTick() - dictData->timeStamp > MULTI_SENTENCE_TIME_WINDOW ||
          dictData->aisData.dataLength + aisBinaryLength > MAX_LENGTH ||
          sentenceNumber > totalSentenceSegments) {
        NMEA_DEBUG_PRINT(
            "[AIS] drop multi seq=%u part=%u/%u last=%u ch=%c expected_ch=%c "
            "age=%lu len=%u\r\n",
            sequentialMessageIdentifier, sentenceNumber, totalSentenceSegments,
            dictData->lastSentenceIndex, vhfChannel, dictData->vhfChannel,
            (unsigned long)(HAL_GetTick() - dictData->timeStamp),
            dictData->aisData.dataLength + aisBinaryLength);
        resetTimeStamp(dictData);
        return NULL;
      }

      // Copy the data and return if it is complete data
      memcpy(&dictData->aisData.sixBitData[dictData->aisData.dataLength],
             aisBinary, aisBinaryLength);
      dictData->aisData.dataLength += aisBinaryLength;
      if (sentenceNumber == totalSentenceSegments) {
        NMEA_DEBUG_PRINT(
            "[AIS] multi complete seq=%u len=%u msg_id=%u mmsi=%lu\r\n",
            sequentialMessageIdentifier, dictData->aisData.dataLength,
            AIS__getMessageID(&dictData->aisData),
            (unsigned long)AIS__getMMSINumber(&dictData->aisData));
        return &dictData->aisData;
      }
      NMEA_DEBUG_PRINT("[AIS] multi progress seq=%u part=%u/%u len=%u\r\n",
                       sequentialMessageIdentifier, sentenceNumber,
                       totalSentenceSegments, dictData->aisData.dataLength);
    }
  } else {
    NMEA_DEBUG_PRINT("[AIS] ignored sentence type=0x%06lX payload=%p\r\n",
                     (unsigned long)sentence_type, (void *)aisBinary);
  }
  return NULL;
}

bool AIS__isSizeMessage(AIS_DATA *self) {
  if (AIS__getMessageID(self) == 5 || AIS__getMessageID(self) == 19 ||
      AIS__getMessageID(self) == 24)
    return true;
  return false;
}

bool AIS__isDynamicMessage(AIS_DATA *self) {
  if (AIS__getMessageID(self) == 1 || AIS__getMessageID(self) == 2 ||
      AIS__getMessageID(self) == 3 || AIS__getMessageID(self) == 18)
    return true;
  return false;
}

bool AIS__isSupportedMessage(AIS_DATA *self) {
  if (AIS__isSizeMessage(self) == true || AIS__isDynamicMessage(self) == true)
    return true;
  return false;
}

bool AIS__checkLength(AIS_DATA *self) {
  uint8_t messageID = AIS__getMessageID(self);
  if ((messageID <= 3 && messageID != 0) || messageID == 18) {
    return self->dataLength == 28;
  } else if (messageID == 5) {
    return self->dataLength == 71;
  } else if (messageID == 19) {
    return self->dataLength == 52;
  } else if (messageID == 24) {
    return self->dataLength == 55;
  }
  return false;
}

uint8_t AIS__getMessageID(AIS_DATA *self) {
  return convertSixBit(self->sixBitData[0]);
}

uint8_t AIS__getRepeatIndicator(AIS_DATA *self) {
  return (convertSixBit(self->sixBitData[1]) >> 4) & 3;
}

uint32_t AIS__getMMSINumber(AIS_DATA *self) {
  return getBinaryBits(self->sixBitData, 8, 37);
}

uint8_t AIS__getNavigationalStatus(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) <= 3 &&
      convertSixBit(self->sixBitData[0]) != 0) {
    return convertSixBit(self->sixBitData[6]) & 15;
  } else {
    return UINT8_MAX;
  }
}

int8_t AIS__getRateOfTurn(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) <= 3 &&
      convertSixBit(self->sixBitData[0]) != 0) {
    return getBinaryBits(self->sixBitData, 42, 49);
  } else {
    return INT8_MIN;
  }
}

uint16_t AIS__getSpeedOverGround(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0) {
    if (convertSixBit(self->sixBitData[0]) <= 3) {
      return getBinaryBits(self->sixBitData, 50, 59);
    } else if (convertSixBit(self->sixBitData[0]) == 18 ||
               convertSixBit(self->sixBitData[0]) == 19) {
      return getBinaryBits(self->sixBitData, 46, 55);
    }
  }
  return UINT16_MAX;
}

uint8_t AIS__getPositionalAccuracy(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0) {
    if (convertSixBit(self->sixBitData[0]) <= 3) {
      return convertSixBit(self->sixBitData[10]) >> 5;
    } else if (convertSixBit(self->sixBitData[0]) == 18 ||
               convertSixBit(self->sixBitData[0]) == 19) {
      return (convertSixBit(self->sixBitData[9]) >> 3) & 1;
    }
  }
  return UINT8_MAX;
}

int32_t AIS__getLatitude(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0) {
    if (convertSixBit(self->sixBitData[0]) <= 3) {
      return getBinaryBits(self->sixBitData, 89, 115);
    } else if (convertSixBit(self->sixBitData[0]) == 18 ||
               convertSixBit(self->sixBitData[0]) == 19) {
      return getBinaryBits(self->sixBitData, 85, 111);
    }
  }
  return INT32_MAX;
}

int32_t AIS__getLongitude(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0) {
    if (convertSixBit(self->sixBitData[0]) <= 3) {
      return getBinaryBits(self->sixBitData, 61, 88);

    } else if (convertSixBit(self->sixBitData[0]) == 18 ||
               convertSixBit(self->sixBitData[0]) == 19) {
      return getBinaryBits(self->sixBitData, 57, 84);
    }
  }
  return INT32_MAX;
}

uint16_t AIS__getCourseOverGround(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0) {
    if (convertSixBit(self->sixBitData[0]) <= 3) {
      return getBinaryBits(self->sixBitData, 116, 127);
    } else if (convertSixBit(self->sixBitData[0]) == 18 ||
               convertSixBit(self->sixBitData[0]) == 19) {
      return getBinaryBits(self->sixBitData, 112, 123);
    }
  }
  return UINT16_MAX;
}

uint16_t AIS__getTrueHeading(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0) {
    if (convertSixBit(self->sixBitData[0]) <= 3) {
      return getBinaryBits(self->sixBitData, 128, 136);
    } else if (convertSixBit(self->sixBitData[0]) == 18 ||
               convertSixBit(self->sixBitData[0]) == 19) {
      return getBinaryBits(self->sixBitData, 124, 132);
    }
  }
  return UINT16_MAX;
}

uint8_t AIS__getTimeStamp(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0) {
    if (convertSixBit(self->sixBitData[0]) <= 3) {
      return getBinaryBits(self->sixBitData, 137, 142);
    } else if (convertSixBit(self->sixBitData[0]) == 18 ||
               convertSixBit(self->sixBitData[0]) == 19) {
      return getBinaryBits(self->sixBitData, 133, 138);
    }
  }
  return UINT8_MAX;
}

uint8_t AIS__getSpecialManeuvreIndicator(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) <= 3 &&
      convertSixBit(self->sixBitData[0]) != 0) {
    return getBinaryBits(self->sixBitData, 143, 144);
  }
  return UINT8_MAX;
}

uint32_t AIS__getCommunicationState(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) != 0 &&
      (convertSixBit(self->sixBitData[0]) <= 3 ||
       convertSixBit(self->sixBitData[0]) == 18)) {
    return getBinaryBits(self->sixBitData, 149, 167);
  }
  return UINT32_MAX;
}

uint8_t AIS__getVersionIndicator(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return (convertSixBit(self->sixBitData[6]) >> 2) & 3;
  }
  return UINT8_MAX;
}

uint32_t AIS__getIMONumber(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return getBinaryBits(self->sixBitData, 40, 69);
  }
  return UINT32_MAX;
}

bool AIS__getCallSign(AIS_DATA *self, uint8_t output[8]) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    getAsciiString(self->sixBitData, output, 70, 111);
    output[7] = '\0';
    return true;
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    getAsciiString(self->sixBitData, output, 90, 131);
    output[7] = '\0';
    return true;
  }
  return false;
}

bool AIS__getName(AIS_DATA *self, uint8_t output[21]) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    getAsciiString(self->sixBitData, output, 112, 231);
    output[20] = '\0';
    return true;
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 0) {
    getAsciiString(self->sixBitData, output, 40, 159);
    output[20] = '\0';
    return true;
  }
  return false;
}

uint8_t AIS__getCargoType(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return getBinaryBits(self->sixBitData, 232, 239);
  } else if (convertSixBit(self->sixBitData[0]) == 19) {
    return getBinaryBits(self->sixBitData, 263, 270);
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    return getBinaryBits(self->sixBitData, 40, 47);
  }
  return UINT8_MAX;
}

uint8_t AIS__getDimensionD(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return convertSixBit(self->sixBitData[44]);
  } else if (convertSixBit(self->sixBitData[0]) == 19) {
    return getBinaryBits(self->sixBitData, 295, 300);
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    return convertSixBit(self->sixBitData[26]);
  }
  return UINT8_MAX;
}

uint8_t AIS__getDimensionC(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return convertSixBit(self->sixBitData[43]);
  } else if (convertSixBit(self->sixBitData[0]) == 19) {
    return getBinaryBits(self->sixBitData, 289, 294);
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    return convertSixBit(self->sixBitData[25]);
  }
  return UINT8_MAX;
}

uint16_t AIS__getDimensionB(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return getBinaryBits(self->sixBitData, 249, 257);
  } else if (convertSixBit(self->sixBitData[0]) == 19) {
    return getBinaryBits(self->sixBitData, 280, 288);
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    return getBinaryBits(self->sixBitData, 141, 149);
  }
  return UINT16_MAX;
}

uint16_t AIS__getDimensionA(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return getBinaryBits(self->sixBitData, 240, 248);
  } else if (convertSixBit(self->sixBitData[0]) == 19) {
    return getBinaryBits(self->sixBitData, 271, 279);
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    return getBinaryBits(self->sixBitData, 132, 140);
  }
  return UINT16_MAX;
}

uint8_t AIS__getPositionFixingDevice(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return (convertSixBit(self->sixBitData[45]) >> 2) & 15;
  } else if (convertSixBit(self->sixBitData[0]) == 19) {
    return (convertSixBit(self->sixBitData[50]) >> 1) & 15;
  } else if (convertSixBit(self->sixBitData[0]) == 24 &&
             (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    return (convertSixBit(self->sixBitData[27]) >> 2) & 15;
  }
  return UINT8_MAX;
}

uint32_t AIS__getETA(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return getBinaryBits(self->sixBitData, 274, 293);
  }
  return UINT32_MAX;
}

uint8_t AIS__getMaximumDraught(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return getBinaryBits(self->sixBitData, 294, 301);
  }
  return 0;
}

bool AIS__getDestination(AIS_DATA *self, uint8_t output[21]) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    getAsciiString(self->sixBitData, output, 302, 421);
    output[20] = '\0';
    return true;
  }
  return false;
}

uint8_t AIS__getDTE(AIS_DATA *self) {
  if (convertSixBit(self->sixBitData[0]) == 5) {
    return (convertSixBit(self->sixBitData[70]) >> 3) & 1;
  }
  return UINT8_MAX;
}

bool AIS__getVendorID(AIS_DATA *self, uint8_t output[8]) {
  if (convertSixBit(self->sixBitData[0]) == 24 &&
      (convertSixBit(self->sixBitData[6]) & 12) == 4) {
    getAsciiString(self->sixBitData, output, 48, 89);
    output[7] = '\0';
    return true;
  }
  return false;
}

/**
 * Transmit a single AIS ship over CAN (0x060).
 * As defined in the sailbot confluence definition.
 * https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1827176527/CAN+Frames
 *
 * @param data AIS data to transmit.
 * @param ship_idx Index of this ship [0, total_ships - 1].
 * @param total_ships Total ships in this transmit cycle.
 * @param hfdcan1 CAN handle.
 * @return HAL_OK on success or a HAL error code.
 */
HAL_StatusTypeDef AIS__CAN_transmit_single(const AIS_DATA *data,
                                           uint8_t ship_idx,
                                           uint8_t total_ships,
                                           FDCAN_HandleTypeDef *hfdcan1) {
  if (!data || !hfdcan1) {
    NMEA_DEBUG_PRINT("[AIS][CAN] tx single skipped: null input\r\n");
    return HAL_ERROR;
  }

  uint32_t mmsi = AIS__getMMSINumber((AIS_DATA *)data);
  uint32_t latitude = AIS__convertLatitude(AIS__getLatitude((AIS_DATA *)data));
  uint32_t longitude =
      AIS__convertLongitude(AIS__getLongitude((AIS_DATA *)data));

  uint16_t speed_over_ground = AIS__getSpeedOverGround((AIS_DATA *)data);
  if (speed_over_ground == UINT16_MAX) {
    speed_over_ground = AIS_SOG_UNAVAILABLE;
  }
  uint16_t course_over_ground = AIS__getCourseOverGround((AIS_DATA *)data);
  if (course_over_ground == UINT16_MAX) {
    course_over_ground = AIS_COG_UNAVAILABLE;
  }
  uint16_t heading = AIS__getTrueHeading((AIS_DATA *)data);
  if (heading == UINT16_MAX) {
    heading = AIS_HEADING_UNAVAILABLE;
  }
  int8_t rate_of_turn = AIS__getRateOfTurn((AIS_DATA *)data);
  if (rate_of_turn == INT8_MIN) {
    rate_of_turn = AIS_ROT_UNAVAILABLE;
  }

  uint16_t dimension_a = AIS__getDimensionA((AIS_DATA *)data);
  uint16_t dimension_b = AIS__getDimensionB((AIS_DATA *)data);
  uint8_t dimension_c = AIS__getDimensionC((AIS_DATA *)data);
  uint8_t dimension_d = AIS__getDimensionD((AIS_DATA *)data);

  uint16_t length = 0;
  uint16_t width = 0;
  if (dimension_a != UINT16_MAX && dimension_b != UINT16_MAX) {
    length = (uint16_t)(dimension_a + dimension_b);
  }
  if (dimension_c != UINT8_MAX && dimension_d != UINT8_MAX) {
    width = (uint16_t)(dimension_c + dimension_d);
  }

  uint8_t payload[32];
  memset(payload, 0, sizeof(payload));

  payload[0] = (uint8_t)(mmsi & 0xFF);
  payload[1] = (uint8_t)((mmsi >> 8) & 0xFF);
  payload[2] = (uint8_t)((mmsi >> 16) & 0xFF);
  payload[3] = (uint8_t)((mmsi >> 24) & 0xFF);

  payload[4] = (uint8_t)(latitude & 0xFF);
  payload[5] = (uint8_t)((latitude >> 8) & 0xFF);
  payload[6] = (uint8_t)((latitude >> 16) & 0xFF);
  payload[7] = (uint8_t)((latitude >> 24) & 0xFF);

  payload[8] = (uint8_t)(longitude & 0xFF);
  payload[9] = (uint8_t)((longitude >> 8) & 0xFF);
  payload[10] = (uint8_t)((longitude >> 16) & 0xFF);
  payload[11] = (uint8_t)((longitude >> 24) & 0xFF);

  payload[12] = (uint8_t)(speed_over_ground & 0xFF);
  payload[13] = (uint8_t)((speed_over_ground >> 8) & 0xFF);

  payload[14] = (uint8_t)(course_over_ground & 0xFF);
  payload[15] = (uint8_t)((course_over_ground >> 8) & 0xFF);

  payload[16] = (uint8_t)(heading & 0xFF);
  payload[17] = (uint8_t)((heading >> 8) & 0xFF);

  payload[18] = (uint8_t)rate_of_turn;

  payload[19] = (uint8_t)(length & 0xFF);
  payload[20] = (uint8_t)((length >> 8) & 0xFF);

  payload[21] = (uint8_t)(width & 0xFF);
  payload[22] = (uint8_t)((width >> 8) & 0xFF);

  payload[23] = ship_idx;
  payload[24] = total_ships;

  NMEA_DEBUG_PRINT(
      "[AIS][CAN] tx ship idx=%u/%u mmsi=%lu lat=%lu lon=%lu sog=%u cog=%u "
      "hdg=%u rot=%d len=%u wid=%u\r\n",
      ship_idx, total_ships, (unsigned long)mmsi, (unsigned long)latitude,
      (unsigned long)longitude, speed_over_ground, course_over_ground, heading,
      rate_of_turn, length, width);

  HAL_StatusTypeDef status = CAN_Transmit(AIS_FRAME_ID, FDCAN_STANDARD_ID,
                                          AIS_FRAME_LENGTH, payload, hfdcan1);
  NMEA_DEBUG_PRINT("[AIS][CAN] tx single status=%d\r\n", status);
  return status;
}

/**
 * Transmit all AIS ships in one batch over CAN (0x060).
 * As defined in the sailbot confluence definition.
 * https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1827176527/CAN+Frames
 *
 * @param data Array of AIS data to transmit.
 * @param ship_count Number of AIS data entries in the array.
 * @param hfdcan1 CAN handle.
 * @return HAL_OK on success or a HAL error code.
 */
HAL_StatusTypeDef AIS__CAN_transmit(const AIS_DATA *data, uint16_t ship_count,
                                    FDCAN_HandleTypeDef *hfdcan1) {
  if (!data || !hfdcan1 || ship_count == 0U) {
    NMEA_DEBUG_PRINT("[AIS][CAN] tx batch skipped data=%p hfdcan=%p count=%u\r\n",
                     (void *)data, (void *)hfdcan1, ship_count);
    return HAL_ERROR;
  }

  NMEA_DEBUG_PRINT("[AIS][CAN] tx batch count=%u\r\n", ship_count);

  for (uint16_t batch_start = 0U; batch_start < ship_count;
       batch_start += AIS_MAX_SHIPS_PER_BATCH) {
    uint16_t remaining = ship_count - batch_start;
    uint8_t total_ships = (remaining > AIS_MAX_SHIPS_PER_BATCH)
                              ? AIS_MAX_SHIPS_PER_BATCH
                              : (uint8_t)remaining;

    for (uint8_t ship_idx = 0; ship_idx < total_ships; ship_idx++) {
      HAL_StatusTypeDef status = AIS__CAN_transmit_single(
          &data[batch_start + ship_idx], ship_idx, total_ships, hfdcan1);
      if (status != HAL_OK) {
        NMEA_DEBUG_PRINT(
            "[AIS][CAN] tx batch failed at global_idx=%u status=%d\r\n",
            (unsigned int)(batch_start + ship_idx), status);
        return status;
      }
    }
  }

  NMEA_DEBUG_PRINT("[AIS][CAN] tx batch complete\r\n");
  return HAL_OK;
}

HAL_StatusTypeDef AIS__CAN_transmit_empty(FDCAN_HandleTypeDef *hfdcan1) {
  if (!hfdcan1) {
    NMEA_DEBUG_PRINT("[AIS][CAN] tx empty skipped: null can handle\r\n");
    return HAL_ERROR;
  }

  uint8_t payload[32];
  memset(payload, 0, sizeof(payload));
  payload[23] = 0U; // ship_idx
  payload[24] = 0U; // total_ships

  HAL_StatusTypeDef status = CAN_Transmit(AIS_FRAME_ID, FDCAN_STANDARD_ID,
                                          AIS_FRAME_LENGTH, payload, hfdcan1);
  NMEA_DEBUG_PRINT("[AIS][CAN] tx empty status=%d\r\n", status);
  return status;
}

/**
 * Process a single AIS message, buffer ships, and transmit when the cycle ends.
 *
 * @param batch AIS CAN batch storage.
 * @param data AIS data to buffer.
 * @param now_ms Current time in milliseconds.
 * @param hfdcan1 CAN handle.
 * @return HAL_OK on success or a HAL error code.
 */
HAL_StatusTypeDef AIS__CAN_process(AIS_CAN_BATCH *batch, const AIS_DATA *data,
                                   uint32_t now_ms,
                                   FDCAN_HandleTypeDef *hfdcan1) {
  if (!batch || !data || !hfdcan1) {
    NMEA_DEBUG_PRINT("[AIS][CAN] process skipped: null input\r\n");
    return HAL_ERROR;
  }

  if (batch->next_send_ms == 0U) {
    batch->next_send_ms = now_ms + AIS_CAN_BATCH_INTERVAL_MS;
    NMEA_DEBUG_PRINT("[AIS][CAN] start batch window now=%lu send_at=%lu\r\n",
                     (unsigned long)now_ms, (unsigned long)batch->next_send_ms);
  }

  uint32_t mmsi = AIS__getMMSINumber((AIS_DATA *)data);
  int ship_index = AIS__findShipIndex(batch, mmsi);
  if (ship_index >= 0) {
    batch->ships[ship_index] = *data;
    NMEA_DEBUG_PRINT("[AIS][CAN] update ship idx=%d mmsi=%lu\r\n", ship_index,
                     (unsigned long)mmsi);
  } else {
    if (batch->ship_count >= AIS_CAN_MAX_SHIPS) {
      NMEA_DEBUG_PRINT("[AIS][CAN] drop mmsi=%lu batch full=%u\r\n",
                       (unsigned long)mmsi, batch->ship_count);
      return HAL_ERROR;
    }
    batch->ships[batch->ship_count] = *data;
    batch->ship_count++;
    NMEA_DEBUG_PRINT("[AIS][CAN] add ship idx=%u mmsi=%lu\r\n",
                     (unsigned int)(batch->ship_count - 1U),
                     (unsigned long)mmsi);
  }

  if ((int32_t)(now_ms - batch->next_send_ms) >= 0) {
    NMEA_DEBUG_PRINT("[AIS][CAN] flush batch now=%lu count=%u\r\n",
                     (unsigned long)now_ms, batch->ship_count);
    HAL_StatusTypeDef status =
        AIS__CAN_transmit(batch->ships, batch->ship_count, hfdcan1);
    batch->ship_count = 0U;
    batch->next_send_ms = now_ms + AIS_CAN_BATCH_INTERVAL_MS;
    NMEA_DEBUG_PRINT("[AIS][CAN] next window send_at=%lu status=%d\r\n",
                     (unsigned long)batch->next_send_ms, status);
    return status;
  }

  NMEA_DEBUG_PRINT("[AIS][CAN] hold batch now=%lu send_at=%lu count=%u\r\n",
                   (unsigned long)now_ms, (unsigned long)batch->next_send_ms,
                   batch->ship_count);
  return HAL_OK;
}
