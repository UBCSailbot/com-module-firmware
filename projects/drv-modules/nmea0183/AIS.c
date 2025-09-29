/*
 * AIS.c
 *
 *  Created on: Apr 21, 2024
 *      Author: Michael Greenough
 */

#include "AIS.h"
#include <stdlib.h>
#include <string.h>

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PFP ---------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Converts from AIS specified 6-bit character into regular ASCII character.
 *
 * @param input Any valid 6-bit AIS character.
 * @return The corresponding valid ASCII character.
 */
uint8_t convertAscii(uint8_t input);

/*
 * Retrieves the bit sequence from the data based on a specified interval. The input data is AIS specified 6-bit character string.
 *
 * @param data[] An array of 6-bit AIS data.
 * @param startPosition The starting bit position of the bit sequence. Must be between 0 and a valid bit in the sequence (inclusive.)
 * @param endPosition The ending bit position of the bit sequence. Must be between startPosition and a valid bit in the sequence (inclusive.)
 * @return The concatenated bit sequence as a 32 bit word.
 */
uint32_t getBinaryBits(uint8_t data[], uint16_t startPosition, uint16_t endPosition);

/*
 * Convert from a 6-bit string to a regular ASCII string.
 *
 * @param input[] An array of 6-bit AIS data.
 * @param output[] The output ASCII string terminated with \0. Must be of size at least (startBit-endBit) / 2 + 1
 * @param startBit The starting bit of the data to take the string from.
 * @param endBit The ending bit of the data to take the string from.
 */
void getAsciiString(uint8_t input[], uint8_t output[], uint16_t startBit, uint16_t endBit);

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
void resetTimeStamp(AIS_MULTI_SENTENCE * aisMultiData);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void AIS__init(AIS_PARSER* self) {
	memset(self, 0, sizeof(AIS_PARSER));
	for(uint8_t i = 0; i < 10; i++){
		resetTimeStamp(&self->multiSentenceHeap[i]);
	}
}

AIS_PARSER* AIS__create() {
    AIS_PARSER* result = (AIS_PARSER*)malloc(sizeof(AIS_PARSER));
    AIS__init(result);
    return result;
}

void AIS__reset(AIS_PARSER* self) {
	AIS__init(self);
}

void AIS__destroy(AIS_PARSER * data) {
    if (data) {
        AIS__reset(data);
        free(data);
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- HELPER FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void resetTimeStamp(AIS_MULTI_SENTENCE * aisMultiData){
	aisMultiData->timeStamp = 0xFFFF0000 - MULTI_SENTENCE_TIME_WINDOW;
}

uint8_t convertSixBit(uint8_t input) {
    if (input <= 87) {
        return input - 48;
    }
    else if (input <= 119) {
        return input - 56;
    }
    else {
        return 255;
    }
}

uint8_t convertAscii(uint8_t input) {
    if (input < 40) {
        return input + 64;
    }
    else {
        return input;
    }
}

uint32_t getBinaryBits(uint8_t data[], uint16_t startPosition, uint16_t endPosition) {
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

void getAsciiString(uint8_t input[], uint8_t output[], uint16_t startBit, uint16_t endBit) {
    for (uint16_t i = startBit; i < endBit; i += 6) {
        output[(i - startBit) / 6] = convertAscii(getBinaryBits(input, i, i + 5));
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- DATA PARSING FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

AIS_DATA * AIS__parseNMEAMessage(AIS_PARSER * self, NMEA0183Raw * data){
	uint8_t * aisBinary = NMEA0183__getField(data, 5);

	if(NMEA0183__getScentenceType(data) == MESSAGE_VDM && aisBinary != NULL){
		uint8_t totalSentenceSegments = NMEA0183__getField(data, 1)[0] - '0';
		uint8_t aisBinaryLength = strlen((char *)aisBinary);


		if(totalSentenceSegments == 1){
			//If this is a single sentence message then just copy it and return it.
			self->singleSentenceData.dataLength = aisBinaryLength;
			memcpy(self->singleSentenceData.sixBitData, aisBinary, self->singleSentenceData.dataLength);
			return &self->singleSentenceData;
		} else {
			uint8_t sequentialMessageIdentifier = NMEA0183__getField(data, 3)[0] - '0';

			//Check that the sequential identifier is in range, or else we will access unallocated memory
			if(sequentialMessageIdentifier > 9)
				return NULL;


			AIS_MULTI_SENTENCE * dictData = &self->multiSentenceHeap[sequentialMessageIdentifier];
			uint8_t vhfChannel = NMEA0183__getField(data, 4)[0];
			uint8_t sentenceNumber = NMEA0183__getField(data, 2)[0] - '0';

			//If this is the first part of the message then update the dictionary with it
			if(sentenceNumber == 1){
				dictData->timeStamp = HAL_GetTick();
				dictData->totalSentenceParts = totalSentenceSegments;
				dictData->lastSentenceIndex = 0;
				dictData->aisData.dataLength = 0;
				dictData->vhfChannel = vhfChannel;
			}

			//Make sure the dictionary element matches the incoming message
			if(dictData->vhfChannel != vhfChannel
					|| ++dictData->lastSentenceIndex != sentenceNumber
					|| dictData->totalSentenceParts != totalSentenceSegments
					|| HAL_GetTick() - dictData->timeStamp > MULTI_SENTENCE_TIME_WINDOW
					|| dictData->aisData.dataLength + aisBinaryLength > MAX_LENGTH
					|| sentenceNumber > totalSentenceSegments){
				resetTimeStamp(dictData);
				return NULL;
			}

			//Copy the data and return if it is complete data
			memcpy(&dictData->aisData.sixBitData[dictData->aisData.dataLength], aisBinary, aisBinaryLength);
			dictData->aisData.dataLength += aisBinaryLength;
			if(sentenceNumber == totalSentenceSegments){
				return &dictData->aisData;
			}
		}
	}
	return NULL;
}

bool AIS__isSizeMessage(AIS_DATA * self){
	if(AIS__getMessageID(self) == 5 || AIS__getMessageID(self) == 19 || AIS__getMessageID(self) == 24)
		return true;
	return false;
}

bool AIS__isDynamicMessage(AIS_DATA * self){
	if(AIS__getMessageID(self) == 1 || AIS__getMessageID(self) == 2 || AIS__getMessageID(self) == 3 || AIS__getMessageID(self) == 18)
		return true;
	return false;
}

bool AIS__isSupportedMessage(AIS_DATA * self){
	if(AIS__isSizeMessage(self) == true || AIS__isDynamicMessage(self) == true)
		return true;
	return false;
}

bool AIS__checkLength(AIS_DATA* self) {
    uint8_t messageID = AIS__getMessageID(self);
    if ((messageID <= 3 && messageID != 0) || messageID == 18) {
        return self->dataLength == 28;
    }
    else if (messageID == 5 ) {
        return self->dataLength == 71;
    }
    else if (messageID == 19) {
        return self->dataLength == 52;
    }
    else if (messageID == 24) {
        return self->dataLength == 55;
    }
    return false;
}

uint8_t AIS__getMessageID(AIS_DATA* self) {
    return convertSixBit(self->sixBitData[0]);
}

uint8_t AIS__getRepeatIndicator(AIS_DATA* self) {
    return (convertSixBit(self->sixBitData[1]) >> 4) & 3;
}

uint32_t AIS__getMMSINumber(AIS_DATA* self) {
    return getBinaryBits(self->sixBitData, 8, 37);
}

uint8_t AIS__getNavigationalStatus(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) <= 3 && convertSixBit(self->sixBitData[0]) != 0) {
        return convertSixBit(self->sixBitData[6]) & 15;
    }
    else {
        return UINT8_MAX;
    }
}

int8_t AIS__getRateOfTurn(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) <= 3 && convertSixBit(self->sixBitData[0]) != 0) {
        return getBinaryBits(self->sixBitData, 42, 49);
    }
    else {
        return INT8_MIN;
    }
}

uint16_t AIS__getSpeedOverGround(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0) {
        if (convertSixBit(self->sixBitData[0]) <= 3) {
            return getBinaryBits(self->sixBitData, 50, 59);
        }
        else if (convertSixBit(self->sixBitData[0]) == 18 || convertSixBit(self->sixBitData[0]) == 19) {
            return getBinaryBits(self->sixBitData, 46, 55);
        }
    }
    return UINT16_MAX;
}

uint8_t AIS__getPositionalAccuracy(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0) {
        if (convertSixBit(self->sixBitData[0]) <= 3) {
            return convertSixBit(self->sixBitData[10]) >> 5;
        }
        else if (convertSixBit(self->sixBitData[0]) == 18 || convertSixBit(self->sixBitData[0]) == 19) {
            return (convertSixBit(self->sixBitData[9]) >> 3) & 1;
        }
    }
    return UINT8_MAX;
}

int32_t AIS__getLatitude(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0) {
        if (convertSixBit(self->sixBitData[0]) <= 3) {
            return getBinaryBits(self->sixBitData, 89, 115);
        }
        else if (convertSixBit(self->sixBitData[0]) == 18 || convertSixBit(self->sixBitData[0]) == 19) {
            return getBinaryBits(self->sixBitData, 85, 111);
        }
    }
    return INT32_MAX;
}

int32_t AIS__getLongitude(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0) {
        if (convertSixBit(self->sixBitData[0]) <= 3) {
            return getBinaryBits(self->sixBitData, 61, 88);

        }
        else if (convertSixBit(self->sixBitData[0]) == 18 || convertSixBit(self->sixBitData[0]) == 19) {
            return getBinaryBits(self->sixBitData, 57, 84);
        }
    }
    return INT32_MAX;
}

uint16_t AIS__getCourseOverGround(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0) {
        if (convertSixBit(self->sixBitData[0]) <= 3) {
            return getBinaryBits(self->sixBitData, 116, 127);
        }
        else if (convertSixBit(self->sixBitData[0]) == 18 || convertSixBit(self->sixBitData[0]) == 19) {
            return getBinaryBits(self->sixBitData, 112, 123);
        }
    }
    return UINT16_MAX;
}

uint16_t AIS__getTrueHeading(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0) {
        if (convertSixBit(self->sixBitData[0]) <= 3) {
            return getBinaryBits(self->sixBitData, 128, 136);
        }
        else if (convertSixBit(self->sixBitData[0]) == 18 || convertSixBit(self->sixBitData[0]) == 19) {
            return getBinaryBits(self->sixBitData, 124, 132);
        }
    }
    return UINT16_MAX;
}

uint8_t AIS__getTimeStamp(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0) {
        if (convertSixBit(self->sixBitData[0]) <= 3) {
            return getBinaryBits(self->sixBitData, 137, 142);
        }
        else if (convertSixBit(self->sixBitData[0]) == 18 || convertSixBit(self->sixBitData[0]) == 19) {
            return getBinaryBits(self->sixBitData, 133, 138);
        }
    }
    return UINT8_MAX;
}

uint8_t AIS__getSpecialManeuvreIndicator(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) <= 3 && convertSixBit(self->sixBitData[0]) != 0) {
        return getBinaryBits(self->sixBitData, 143, 144);
    }
    return UINT8_MAX;
}

uint32_t AIS__getCommunicationState(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) != 0 && (convertSixBit(self->sixBitData[0]) <= 3 || convertSixBit(self->sixBitData[0]) == 18)) {
        return getBinaryBits(self->sixBitData, 149, 167);
    }
    return UINT32_MAX;
}

uint8_t AIS__getVersionIndicator(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return (convertSixBit(self->sixBitData[6]) >> 2) & 3;
    }
    return UINT8_MAX;
}

uint32_t AIS__getIMONumber(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return getBinaryBits(self->sixBitData, 40, 69);
    }
    return UINT32_MAX;
}

bool AIS__getCallSign(AIS_DATA* self, uint8_t output[8]) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        getAsciiString(self->sixBitData, output, 70, 111);
        output[7] = '\0';
        return true;
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        getAsciiString(self->sixBitData, output, 90, 131);
        output[7] = '\0';
        return true;
    }
    return false;
}

bool AIS__getName(AIS_DATA* self, uint8_t output[21]) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        getAsciiString(self->sixBitData, output, 112, 231);
        output[20] = '\0';
        return true;
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 0) {
        getAsciiString(self->sixBitData, output, 40, 159);
        output[20] = '\0';
        return true;
    }
    return false;
}

uint8_t AIS__getCargoType(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return getBinaryBits(self->sixBitData, 232, 239);
    }
    else if (convertSixBit(self->sixBitData[0]) == 19) {
        return getBinaryBits(self->sixBitData, 263, 270);
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        return getBinaryBits(self->sixBitData, 40, 47);
    }
    return UINT8_MAX;
}

uint8_t AIS__getDimensionD(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return convertSixBit(self->sixBitData[44]);
    }
    else if (convertSixBit(self->sixBitData[0]) == 19) {
        return getBinaryBits(self->sixBitData, 295, 300);
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        return convertSixBit(self->sixBitData[26]);
    }
    return UINT8_MAX;
}

uint8_t AIS__getDimensionC(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return convertSixBit(self->sixBitData[43]);
    }
    else if (convertSixBit(self->sixBitData[0]) == 19) {
        return getBinaryBits(self->sixBitData, 289, 294);
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        return convertSixBit(self->sixBitData[25]);

    }
    return UINT8_MAX;
}

uint16_t AIS__getDimensionB(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return getBinaryBits(self->sixBitData, 249, 257);
    }
    else if (convertSixBit(self->sixBitData[0]) == 19) {
        return getBinaryBits(self->sixBitData, 280, 288);
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        return getBinaryBits(self->sixBitData, 141, 149);

    }
    return UINT16_MAX;
}

uint16_t AIS__getDimensionA(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return getBinaryBits(self->sixBitData, 240, 248);
    }
    else if (convertSixBit(self->sixBitData[0]) == 19) {
        return getBinaryBits(self->sixBitData, 271, 279);
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        return getBinaryBits(self->sixBitData, 132, 140);

    }
    return UINT16_MAX;
}

uint8_t AIS__getPositionFixingDevice(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return (convertSixBit(self->sixBitData[45]) >> 2) & 15;
    }
    else if (convertSixBit(self->sixBitData[0]) == 19) {
        return (convertSixBit(self->sixBitData[50]) >> 1) & 15;
    }
    else if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        return (convertSixBit(self->sixBitData[27]) >> 2) & 15;
    }
    return UINT8_MAX;
}

uint32_t AIS__getETA(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return getBinaryBits(self->sixBitData, 274, 293);
    }
    return UINT32_MAX;
}

uint8_t AIS__getMaximumDraught(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return getBinaryBits(self->sixBitData, 294, 301);
    }
    return 0;
}

bool AIS__getDestination(AIS_DATA* self, uint8_t output[21]) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        getAsciiString(self->sixBitData, output, 302, 421);
        output[20] = '\0';
        return true;
    }
    return false;
}

uint8_t AIS__getDTE(AIS_DATA* self) {
    if (convertSixBit(self->sixBitData[0]) == 5) {
        return (convertSixBit(self->sixBitData[70]) >> 3) & 1;

    }
    return UINT8_MAX;
}

bool AIS__getVendorID(AIS_DATA* self, uint8_t output[8]) {
    if (convertSixBit(self->sixBitData[0]) == 24 && (convertSixBit(self->sixBitData[6]) & 12) == 4) {
        getAsciiString(self->sixBitData, output, 48, 89);
        output[7] = '\0';
        return true;
    }
    return false;
}
