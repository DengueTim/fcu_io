#ifndef BLACKBOX_PARSER_H_
#define BLACKBOX_PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "blackbox_fielddefs.h"
#include "parser_input_stream.h"

#define FLIGHT_LOG_MAX_LOGS_IN_FILE 31
#define FLIGHT_LOG_MAX_FIELDS 128
#define FLIGHT_LOG_MAX_FRAME_LENGTH 256

#define FLIGHT_LOG_FIELD_INDEX_ITERATION 0
#define FLIGHT_LOG_FIELD_INDEX_TIME 1

#define FLIGHT_LOG_MAX_MOTORS 8
#define FLIGHT_LOG_MAX_SERVOS 8

namespace blackbox {

class Parser {
public:
	typedef enum FirmwareType {
		FIRMWARE_TYPE_UNKNOWN = 0, FIRMWARE_TYPE_BASEFLIGHT, FIRMWARE_TYPE_CLEANFLIGHT
	} FirmwareType;

	typedef struct flightLogFrameStatistics_t {
		uint32_t bytes;
		// Frames decoded to the right length and had reasonable data in them:
		uint32_t validCount;

		// Frames decoded to the right length but the data looked bad so they were rejected, or stream was desynced from previous lost frames:
		uint32_t desyncCount;

		// Frames didn't decode to the right length at all
		uint32_t corruptCount;

		uint32_t sizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH + 1];
	} flightLogFrameStatistics_t;

	typedef struct flightLogFieldStatistics_t {
		int64_t min, max;
	} flightLogFieldStatistics_t;

	typedef struct flightLogStatistics_t {
		// Number of frames that failed to decode:
		uint32_t totalCorruptFrames;

		//If our sampling rate is less than 1, we won't log every loop iteration, and that is accounted for here:
		uint32_t intentionallyAbsentIterations;

		bool haveFieldStats;
		flightLogFieldStatistics_t field[FLIGHT_LOG_MAX_FIELDS];
		flightLogFrameStatistics_t frame[256];
	} flightLogStatistics_t;

	struct flightLogPrivate_t;

	/*
	 * We provide a list of indexes of well-known fields to save callers the trouble of comparing field name strings
	 * to hunt down the fields they're interested in. Absent fields will have index -1.
	 */
	typedef struct gpsGFieldIndexes_t {
		int time;
		int GPS_numSat;
		int GPS_coord[2];
		int GPS_altitude;
		int GPS_speed;
		int GPS_ground_course;
	} gpsGFieldIndexes_t;

	typedef struct gpsHFieldIndexes_t {
		int GPS_home[2];
	} gpsHFieldIndexes_t;

	typedef struct slowFieldIndexes_t {
		int flightModeFlags;
		int stateFlags;
		int failsafePhase;
	} slowFieldIndexes_t;

	typedef struct mainFieldIndexes_t {
		int loopIteration;
		int time;

		int pid[3][3]; //First dimension is [P, I, D], second dimension is axis

		int rcCommand[4];

		int vbatLatest, amperageLatest;
		int magADC[3];
		int BaroAlt;
		int sonarRaw;
		int rssi;

		int gyroADC[3];
		int accSmooth[3];

		int motor[FLIGHT_LOG_MAX_MOTORS];
		int servo[FLIGHT_LOG_MAX_SERVOS];
	} mainFieldIndexes_t;

	/**
	 * Information about the system configuration of the craft being logged (aids in interpretation
	 * of the log data).
	 */
	typedef struct flightLogSysConfig_t {
		int minthrottle, maxthrottle;
		unsigned int rcRate, yawRate;

		// Calibration constants from the hardware sensors:
		uint16_t acc_1G;
		float gyroScale;

		uint8_t vbatscale;
		uint8_t vbatmaxcellvoltage;
		uint8_t vbatmincellvoltage;
		uint8_t vbatwarningcellvoltage;

		int16_t currentMeterOffset, currentMeterScale;

		uint16_t vbatref;

		FirmwareType firmwareType;
	} flightLogSysConfig_t;

	typedef struct flightLogFrameDef_t {
		char *namesLine; // The parser owns this memory to store the field names for this frame type (as a single string)

		int fieldCount;

		char *fieldName[FLIGHT_LOG_MAX_FIELDS];

		int fieldSigned[FLIGHT_LOG_MAX_FIELDS];
		int predictor[FLIGHT_LOG_MAX_FIELDS];
		int encoding[FLIGHT_LOG_MAX_FIELDS];
	} flightLogFrameDef_t;

	virtual void flightLogMetadataReady() = 0;
	virtual void flightLogFrameReady(bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize) = 0;
	virtual void flightLogEventReady(flightLogEvent_t *event) = 0;


	Parser(ParserInputStream &pis);
	virtual ~Parser();

	bool parse(bool raw);

	int flightLogEstimateNumCells();

	unsigned int flightLogVbatADCToMillivolts(uint16_t vbatADC);
	unsigned int flightLogAmperageADCToMilliamps(uint16_t amperageADC);
	double flightlogGyroToRadiansPerSecond(int32_t gyroRaw);
	double flightlogAccelerationRawToGs(int32_t accRaw);
	void flightlogFlightModeToString(uint32_t flightMode, char *dest, int destLen);
	void flightlogFlightStateToString(uint32_t flightState, char *dest, int destLen);
	void flightlogFailsafePhaseToString(uint8_t failsafePhase, char *dest, int destLen);

private:
	ParserInputStream &pis_;

	typedef enum ParserState {
		PARSER_STATE_HEADER = 0, PARSER_STATE_DATA
	} ParserState;

	typedef void (*FlightLogFrameParse)(Parser &parser, bool raw);
	typedef bool (*FlightLogFrameComplete)(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw);

	typedef struct flightLogFrameType_t {
		uint8_t marker;
		FlightLogFrameParse parse;
		FlightLogFrameComplete complete;
	} flightLogFrameType_t;

	flightLogFrameType_t frameTypes_[6];


	flightLogStatistics_t stats_;

	//Information about fields which we need to decode them properly
	flightLogFrameDef_t frameDefs_[256];

	flightLogSysConfig_t sysConfig_;

	unsigned int frameIntervalI_;
	unsigned int frameIntervalPNum_, frameIntervalPDenom_;

	mainFieldIndexes_t mainFieldIndexes_;
	gpsGFieldIndexes_t gpsFieldIndexes_;
	gpsHFieldIndexes_t gpsHomeFieldIndexes_;
	slowFieldIndexes_t slowFieldIndexes_;

	int dataVersion_;

	// Blackbox state:
	int32_t blackboxHistoryRing_[3][FLIGHT_LOG_MAX_FIELDS];

	/* Points into blackboxHistoryRing to give us a circular buffer.
	 *
	 * 0 - space to decode new frames into, 1 - previous frame, 2 - previous previous frame
	 *
	 * Previous frame pointers are NULL when no valid history exists of that age.
	 */
	int32_t* mainHistory_[3];bool mainStreamIsValid_;

	int32_t gpsHomeHistory_[2][FLIGHT_LOG_MAX_FIELDS]; // 0 - space to decode new frames into, 1 - previous frame
	bool gpsHomeIsValid_;

	//Because these events don't depend on previous events, we don't keep copies of the old state, just the current one:
	flightLogEvent_t lastEvent_;
	int32_t lastGPS_[FLIGHT_LOG_MAX_FIELDS];
	int32_t lastSlow_[FLIGHT_LOG_MAX_FIELDS];

	// How many intentionally un-logged frames did we skip over before we decoded the current frame?
	uint32_t lastSkippedFrames_;

	// Details about the last main frame that was successfully parsed
	uint32_t lastMainFrameIteration_;
	uint32_t lastMainFrameTime_;


	bool looksLikeFrameCompleted_;
	bool prematureEof_;

	void identifyFields(uint8_t frameType, flightLogFrameDef_t *frameDef);
	void identifyMainFields(flightLogFrameDef_t *frameDef);
	void identifyGPSFields(flightLogFrameDef_t *frameDef);
	void identifyGPSHomeFields(flightLogFrameDef_t *frameDef);
	void identifySlowFields(flightLogFrameDef_t *frameDef);

	void parseHeaderLine();
	flightLogFrameType_t* getFrameType(uint8_t c);



};

}
#endif
