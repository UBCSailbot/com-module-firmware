/*
 * This library takes in raw AIS and provides methods for the user to get relevant data from it.
 * Currently the library only provides methods to parse message 1, 2, 3, 5, 18, 19, 24A and 24B. 
 *
 * For further information on AIS sentences see:
 *	1. https://www.navcen.uscg.gov/ais-messages
 *
 *	NOTE: THIS PROGRAM IS STRICTLY SINGLE THREADED. DO NOT PUT ANY METHODS IN AN INTERRUPT.
 *
 *  Created on: Apr 21, 2024
 *      Author: Michael Greenough
 */


#ifndef INC_AIS_H_
#define INC_AIS_H_

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- INCLUDES ---------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "NMEA0183.h"

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Defines the max number of bytes the library will parse. Currently this corresponds to message 5.
#define MAX_LENGTH 424

//This is the time duration during which all parts of a multi-part message must be received. Otherwise this message will be dropped.
#define MULTI_SENTENCE_TIME_WINDOW 10000


//The AIS_PARSER data object
typedef struct {
	uint8_t sixBitData[MAX_LENGTH];
	uint16_t dataLength;
} AIS_DATA;

//The AIS_PARSER multi sentence object. This is used in a dictionary to store the segments of any multi-part messages.
typedef struct {
	uint8_t totalSentenceParts;
	uint8_t lastSentenceIndex;
	uint8_t vhfChannel;
	uint32_t timeStamp;
	AIS_DATA aisData;
} AIS_MULTI_SENTENCE;

//The AIS parser object.
typedef struct {
	AIS_DATA singleSentenceData;
	AIS_MULTI_SENTENCE multiSentenceHeap[10];
} AIS_PARSER;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Creates a new AIS_PARSER object.
 *
 * @return an initialized AIS_PARSER object.
 */
AIS_PARSER* AIS__create();

/*
 * Deletes the AIS_PARSER object.
 *
 * @param self Must be an initialized AIS_PARSER object
 */
void AIS__destroy(AIS_PARSER* self);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- AIS_PARSER METHODS ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Parses a NMEA0183 message. It will take a NMEA0183 message and return AIS_DATA
 *
 * @param self is an initialized AIS_PARSER
 * @param data is an initialized NMEA0183 object with valid data
 * @return is a pointer to processed AIS data. Will return NULL if there is no data ready.
 */
AIS_DATA * AIS__parseNMEAMessage(AIS_PARSER * self, NMEA0183Raw * data);

/*
 * Check if the message type include the dimensions of the vessel
 *
 * @param self Is an initialized AIS_DATA object
 * @return Is true is the message includes the dimension of the vessel. false otherwise.
 */
bool AIS__isSizeMessage(AIS_DATA * self);

/*
 * Check if the message is a dynamic message type (where it would include things like latitude and longitude)
 *
 * @param self Is an initialized AIS_DATA object
 * @return Is true is the message is a dynamic message type. false otherwise.
 */
bool AIS__isDynamicMessage(AIS_DATA * self);

/*
 * Check if the message type is supported by this module.
 *
 * @param self Is an initialized AIS_DATA object
 * @return Is true is the message type is supported. false otherwise.
 */
bool AIS__isSupportedMessage(AIS_DATA * self);

/*
 * Check if number of bytes passed is equal to the number expected by the message ID.
 * 
 * @param self Is an initialized AIS_DATA object.
 * @return Is true if the the length is correct or if the message ID is not supported by this library, false otherwise.
 */
bool AIS__checkLength(AIS_DATA* data);

/*
 * Returns the numeric ID of the received message.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The ID value from 1-63.
 */
uint8_t AIS__getMessageID(AIS_DATA* data);

/*
 * Returns the repeat indicator of the message as defined in [1].
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The repeat indicator is the 2 LSBs.
 */
uint8_t AIS__getRepeatIndicator(AIS_DATA* data);

/*
 * Returns the MMSI number of the vessel that broadcasted.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return Returns the MMSI number of the vessel that broadcasted. This is in the 30 LSBs.
 */
uint32_t AIS__getMMSINumber(AIS_DATA* data);

/*
 * Gets the navigational status of the vessel that broadcasted. Only works for message ID of 1,2 or 3 otherwise returns UINT8_MAX. The valid navigational values are:
 *	0 = under way using engine
 *	1 = at anchor
 *	2 = not under command
 *	3 = restricted maneuverability
 *	4 = constrained by her draught
 *	5 = moored
 *	6 = aground
 *	7 = engaged in fishing
 *	8 = under way sailing
 *	9 = reserved for future amendment of navigational status for ships carrying DG, HS, or MP, or IMO hazard or pollutant category C, high speed craft (HSC)
 *	10 = reserved for future amendment of navigational status for ships carrying dangerous goods (DG), harmful substances (HS) or marine pollutants (MP), or IMO hazard or pollutant category A, wing in ground (WIG);
 *	11 = power-driven vessel towing astern (regional use);
 *	12 = power-driven vessel pushing ahead or towing alongside (regional use);
 *	13 = reserved for future use,
 *	14 = AIS-SART (active), MOB-AIS, EPIRB-AIS
 *	15 = undefined = default (also used by AIS-SART, MOB-AIS and EPIRB-AIS under test)
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The navigational status in the 4 LSBs.
 */
uint8_t AIS__getNavigationalStatus(AIS_DATA* data);

/*
 * Returns the rate of turn of the broadcasted vessel. Only works for message ID of 1, 2, or 3 otherwise returns INT8_MIN.
 * For information on how ROT is formated see [1].
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The rate of turn.
 */
int8_t AIS__getRateOfTurn(AIS_DATA* data);

/*
 * Returns the speed over ground of the broadcasted vessel. Only works for message ID of 1, 2, 3, 18, or 19 otherwise returns UINT16_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The speed over ground in 1/10 knots. 1023 means the data is unavailable. 1022 means a vessel speed of >=102.2 knots. Data in aligned in 10 LSBs.
 */
uint16_t AIS__getSpeedOverGround(AIS_DATA* data);

/*
 * Returns the positional accuracy of the broadcasted vessel. Only works for message ID of 1, 2, 3, 18 or 19 otherwise returns UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The positional accuracy in the LSB. 0 means an accuracy of >10m and is the default value. 1 means an accuracy of <=10m.
 */
uint8_t AIS__getPositionalAccuracy(AIS_DATA* data);

/*
 * Returns the longitude of the broadcasted vessel. Only works for message ID of 1, 2, 3, 18 or 19. Otherwise returns INT32_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The longitude in degrees / 600,000 with East being positive. 181 means data is unavailable.
 */
int32_t AIS__getLongitude(AIS_DATA* data);

/*
 * Returns the latitude of the broadcasted vessel. Only works for message ID of 1, 2, 3, 18 or 19. Otherwise returns INT32_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The longitude in degrees / 600,000 with North being positive. 91 means data is unavailable.
 */
int32_t AIS__getLatitude(AIS_DATA* data);

/*
 * Returns the course over ground of the broadcasted vessel. Only works for message ID of 1, 2, 3, 18 or 19. Otherwise returns UINT16_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The course over ground in degrees 1/10. 3600 means data is unavailable.
 */
uint16_t AIS__getCourseOverGround(AIS_DATA* data);

/*
 * Returns the true heading of the broadcasted vessel. Only works for message ID of 1, 2, 3, 18 or 19. Otherwise returns UINT16_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The true heading in degrees. 511 indicates data is unavailable.
*/
uint16_t AIS__getTrueHeading(AIS_DATA* data);

/*
 * Returns the time stamp of the broadcasted vessel. Only works for message ID of 1, 2, 3, 18 or 19. Otherwise returns UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The UTC second when the report was generated by the electronic position system (EPFS)
 *				0-59 = Regular transmission
 *				60 = Time stamp is not available, which is also the default value
 *				61 = Positioning system is in manual input mode
 *				62 = Electronic position fixing system operates in estimated (dead reckoning) mode
 *				63 = The positioning system is inoperative
 */
uint8_t AIS__getTimeStamp(AIS_DATA* data);

/*
 * Returns the special maneuvre indicator of the broadcasted vessel. Only works for message ID of 1, 2, 3 or 18. Otherwise returns UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The special maneuvre indicator in the 2 LSBs. Values are as follows:
 *				0 = not available / default
 *				1 = not engaged in special maneuver
 *				2 = engaged in special maneuver
 */
uint8_t AIS__getSpecialManeuvreIndicator(AIS_DATA* data);

/*
 * Returns the communication state of the broadcasted vessel. Only works for message ID of 1, 2, 3 or 18. Otherwise returns UINT32_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The communication in the 19 LSBs. See [1] for the formatting.
 */
uint32_t AIS__getCommunicationState(AIS_DATA* data);

/*
 * Returns the version indicator of the broadcasted AIS module. Only works for message ID of 5. Otherwise returns UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The version indicator in the 2 LSBs. Takes the following values:
 *				0 = station compliant with Recommendation ITU-R M.1371-1
 *				1 = station compliant with Recommendation ITU-R M.1371-3 (or later)
 *				2 = station compliant with Recommendation ITU-R M.1371-5 (or later)
 *				3 = station compliant with future editions
 */
uint8_t AIS__getVersionIndicator(AIS_DATA* data);

/*
 * Returns the IMO number of the broadcasted vessel. Only works for message ID of 5. Otherwise returns UINT32_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The IMO number according to:
 *				0 = not available / default – Not applicable to SAR aircraft
 *				0000000001-0000999999 = not used
 *				0001000000-0009999999 = valid IMO number;
 *				0010000000-1073741823 = official flag state number.
 */
uint32_t AIS__getIMONumber(AIS_DATA* data);

/*
 * Sets output to the callsign of the broadcasted ship. Works for message ID of 5 or 24B resulting in a return of true. Otherwise returns false.
 *
 * @param self Is an initialized AIS_DATA object.
 * @param output Will be edited to contain the callsign of the ship in plain ASCII. First 7 bytes is the callsign, last byte is \0.
 * @return Will be true for a valid message ID.
 */
bool AIS__getCallSign(AIS_DATA* data, uint8_t output[8]);

/*
 * Sets output to the name of the broadcasted ship. Works for message ID of 5 or 24A resulting in a return of true. Otherwise returns false.
 *
 * @param self Is an initialized AIS_DATA object.
 * @param output Will be edited to contain the name of the ship in plain ASCII. First 20 bytes is the name, last byte is \0.
 * @return Will be true for a valid message ID.
 */
bool AIS__getName(AIS_DATA* data, uint8_t output[21]);

/*
 * Returns the type of ship / cargo. Works for message ID of 5, 19 or 24B otherwise returns a value of UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return THe type of cargo / ship. See [1] for definition of return values.
 */
uint8_t AIS__getCargoType(AIS_DATA* data);

/*
 * Returns dimension A. Works for message ID of 5, 19 or 24B otherwise returns a value of UINT16_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return Dimension A, the distance from the AIS_PARSER unit to the bow of the ship.
 */
uint16_t AIS__getDimensionA(AIS_DATA* data);

/*
 * Returns dimension B. Works for message ID of 5, 19 or 24B otherwise returns a value of UINT16_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return Dimension B, the distance from the AIS_PARSER unit to the stern of the ship.
 */
uint16_t AIS__getDimensionB(AIS_DATA* data);

/*
 * Returns dimension C. Works for message ID of 5, 19 or 24B otherwise returns a value of UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return Dimension C, the distance from the AIS_PARSER unit to the port side of the ship.
 */
uint8_t AIS__getDimensionC(AIS_DATA* data);

/*
 * Returns dimension D. Works for message ID of 5, 19 or 24B otherwise returns a value of UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return Dimension D, the distance from the AIS unit to the starboard side of the ship.
 */
uint8_t AIS__getDimensionD(AIS_DATA* data);

/*
 * Returns the type of position fixing device. Works for message ID of 5, 19 or 24B otherwise returns a value of UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return Position fixing device type:
 *				0 = undefined (default)
 *				1 = GPS
 *				2 = GLONASS
 *				3 = combined GPS/GLONASS
 *				4 = Loran-C
 *				5 = Chayka
 *				6 = integrated navigation system
 *				7 = surveyed
 *				8 = Galileo,
 *				9-14 = not used
 *				15 = internal GNSS
 */
uint8_t AIS__getPositionFixingDevice(AIS_DATA* data);

/*
 * Returns the ETA. Works for message ID of 5 otherwise returns a value of UINT32_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return The ETA in the following format: MMDDHHMM UTC
 *				Bits 19-16: month; 1-12; 0 = not available = default
 *				Bits 15-11: day; 1-31; 0 = not available = default
 *				Bits 10-6: hour; 0-23; 24 = not available = default
 *				Bits 5-0: minute; 0-59; 60 = not available = default
 */
uint32_t AIS__getETA(AIS_DATA* data);

/*
 * Returns the maximum present static draught. Works for message ID of 5 otherwise returns 0.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return Returns value in 1/10m:
 *				255 = draught 25.5 m or greater
 *				0 = not available = default
 */
uint8_t AIS__getMaximumDraught(AIS_DATA* data);

/*
 * Sets output to the destination of the broadcasted ship. Works for message ID of 5 resulting in a return of true. Otherwise returns false.
 *
 * @param self Is an initialized AIS_DATA object.
 * @param output Will be edited to contain the destination of the ship in plain ASCII. First 20 bytes is the destination, last byte is \0.
 * @return Will be true for a valid message ID.
 */
bool AIS__getDestination(AIS_DATA* data, uint8_t output[21]);

/*
 * Returns if data terminal equipment is available. Works for message ID of 5 otherwise returns UINT8_MAX.
 *
 * @param self Is an initialized AIS_DATA object.
 * @return 0 if it is available or 1 if it is not.
 */
uint8_t AIS__getDTE(AIS_DATA* data);

/*
 * Sets output to the vendor ID of the broadcasted ship. Works for message ID of 24B resulting in a return of true. Otherwise returns false.
 *
 * @param self Is an initialized AIS_DATA object.
 * @param output Will be edited to contain the vendor ID of the ship in plain ASCII. First 7 bytes is the destination, last byte is \0.
 * @return Will be true for a valid message ID.
 */
bool AIS__getVendorID(AIS_DATA* data, uint8_t output[8]);


#endif /* INC_AIS_H_ */
