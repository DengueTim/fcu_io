#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES
#include <math.h>

#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <assert.h>

#include "blackbox/parser.h"
#include "blackbox/tools.h"
#include "blackbox/decoders.h"

namespace blackbox {

#define LOG_START_MARKER "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"

//Assume that even in the most woeful logging situation, we won't miss 10 seconds of frames
#define MAXIMUM_TIME_JUMP_BETWEEN_FRAMES (10 * 1000000)

//Likewise for iteration count
#define MAXIMUM_ITERATION_JUMP_BETWEEN_FRAMES (500 * 10)

static void parseIntraframe(Parser &parser, bool raw);
static void parseInterframe(Parser &parser, bool raw);
static void parseGPSFrame(Parser &parser, bool raw);
static void parseGPSHomeFrame(Parser &parser, bool raw);
static void parseEventFrame(Parser &parser, bool raw);
static void parseSlowFrame(Parser &parser, bool raw);

static bool completeIntraframe(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw);
static bool completeInterframe(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw);
static bool completeEventFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw);
static bool completeGPSFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw);
static bool completeGPSHomeFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw);
static bool completeSlowFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw);

static void resetSysConfigToDefaults(Parser::flightLogSysConfig_t *config) {
	config->minthrottle = 1150;
	config->maxthrottle = 1850;

	config->vbatref = 4095;
	config->vbatscale = 110;
	config->vbatmincellvoltage = 33;
	config->vbatmaxcellvoltage = 43;
	config->vbatwarningcellvoltage = 35;

	config->currentMeterOffset = 0;
	config->currentMeterScale = 400;

	config->rcRate = 90;
	config->yawRate = 0;

	// Default these to silly numbers, because if we don't know the hardware we can't even begin to guess:
	config->acc_1G = 1;
	config->gyroScale = 1;

	config->firmwareType = Parser::FIRMWARE_TYPE_UNKNOWN;
}

Parser::Parser(ParserInputStream &pis) :
		pis_(pis), looksLikeFrameCompleted_(false), prematureEof_(false), gpsHomeIsValid_(false) {

	frameTypes_[0].marker = 'I';
	frameTypes_[0].parse = parseIntraframe;
	frameTypes_[0].complete = completeIntraframe;
	frameTypes_[1].marker = 'P';
	frameTypes_[1].parse = parseInterframe;
	frameTypes_[1].complete = completeInterframe;
	frameTypes_[2].marker = 'G';
	frameTypes_[2].parse = parseGPSFrame;
	frameTypes_[2].complete = completeGPSFrame;
	frameTypes_[3].marker = 'H';
	frameTypes_[3].parse = parseGPSHomeFrame;
	frameTypes_[3].complete = completeGPSHomeFrame;
	frameTypes_[4].marker = 'E';
	frameTypes_[4].parse = parseEventFrame;
	frameTypes_[4].complete = completeEventFrame;
	frameTypes_[5].marker = 'S';
	frameTypes_[5].parse = parseSlowFrame;
	frameTypes_[5].complete = completeSlowFrame;

	mainHistory_[0] = blackboxHistoryRing_[0];
	mainHistory_[1] = NULL;
	mainHistory_[2] = NULL;

	resetSysConfigToDefaults(&sysConfig_);

	frameIntervalI_ = 32;
	frameIntervalPNum_ = 1;
	frameIntervalPDenom_ = 1;

	lastEvent_.event = FLIGHT_LOG_EVENT_UNINITIALIZED;

	/*
	 * Start off all the field indexes as -1 so we can use that as a not-present identifier.
	 * Conveniently, -1 has all bits set so we can just byte-fill
	 */
	memset(&mainFieldIndexes_, (char) 0xFF, sizeof(mainFieldIndexes_));
	memset(&gpsFieldIndexes_, (char) 0xFF, sizeof(gpsFieldIndexes_));
	memset(&gpsHomeFieldIndexes_, (char) 0xFF, sizeof(gpsHomeFieldIndexes_));

	lastSkippedFrames_ = 0;
	lastMainFrameIteration_ = (uint32_t) -1;
	lastMainFrameTime_ = (uint32_t) -1;
}

Parser::~Parser() {

}

/**
 * Parse a comma-separated list of field names into the given frame definition. Sets the fieldCount field based on the
 * number of names parsed.
 */
static void parseFieldNames(const char *line, Parser::flightLogFrameDef_t *frameDef) {
	char *start, *end;
	bool done = false;

	//Make a copy of the line so we can manage its lifetime (and write to it to null terminate the fields)
	frameDef->namesLine = strdup(line);
	frameDef->fieldCount = 0;

	start = frameDef->namesLine;

	while (!done && *start) {
		end = start;

		do {
			end++;
		} while (*end != ',' && *end != 0);

		frameDef->fieldName[frameDef->fieldCount++] = start;

		if (*end == 0) {
			done = true;
		}

		*end = 0;

		start = end + 1;
	}
}

static void parseCommaSeparatedIntegers(char *line, int *target, int maxCount) {
	char *start, *end;
	bool done = false;

	start = line;

	while (!done && *start && maxCount > 0) {
		end = start + 1;

		while (*end != ',' && *end != 0)
			end++;

		if (*end == 0) {
			done = true;
		}

		*end = 0;

		*target = atoi(start);
		target++;
		maxCount--;

		start = end + 1;
	}
}

void Parser::identifyMainFields(Parser::flightLogFrameDef_t *frameDef) {
	int fieldIndex;

	for (fieldIndex = 0; fieldIndex < frameDef->fieldCount; fieldIndex++) {
		const char *fieldName = frameDef->fieldName[fieldIndex];

		if (startsWith(fieldName, "motor[")) {
			int motorIndex = atoi(fieldName + strlen("motor["));

			if (motorIndex >= 0 && motorIndex < FLIGHT_LOG_MAX_MOTORS) {
				mainFieldIndexes_.motor[motorIndex] = fieldIndex;
			}
		} else if (startsWith(fieldName, "rcCommand[")) {
			int rcCommandIndex = atoi(fieldName + strlen("rcCommand["));

			if (rcCommandIndex >= 0 && rcCommandIndex < 4) {
				mainFieldIndexes_.rcCommand[rcCommandIndex] = fieldIndex;
			}
		} else if (startsWith(fieldName, "axis")) {
			int axisIndex = atoi(fieldName + strlen("axisX["));

			switch (fieldName[strlen("axis")]) {
			case 'P':
				mainFieldIndexes_.pid[0][axisIndex] = fieldIndex;
				break;
			case 'I':
				mainFieldIndexes_.pid[1][axisIndex] = fieldIndex;
				break;
			case 'D':
				mainFieldIndexes_.pid[2][axisIndex] = fieldIndex;
				break;
			}
		} else if (startsWith(fieldName, "gyroData[")) {
			int axisIndex = atoi(fieldName + strlen("gyroData["));

			mainFieldIndexes_.gyroADC[axisIndex] = fieldIndex;
		} else if (startsWith(fieldName, "gyroADC[")) {
			int axisIndex = atoi(fieldName + strlen("gyroADC["));

			mainFieldIndexes_.gyroADC[axisIndex] = fieldIndex;
		} else if (startsWith(fieldName, "magADC[")) {
			int axisIndex = atoi(fieldName + strlen("magADC["));

			mainFieldIndexes_.magADC[axisIndex] = fieldIndex;
		} else if (startsWith(fieldName, "accSmooth[")) {
			int axisIndex = atoi(fieldName + strlen("accSmooth["));

			mainFieldIndexes_.accSmooth[axisIndex] = fieldIndex;
		} else if (startsWith(fieldName, "servo[")) {
			int servoIndex = atoi(fieldName + strlen("servo["));

			mainFieldIndexes_.servo[servoIndex] = fieldIndex;
		} else if (strcmp(fieldName, "vbatLatest") == 0) {
			mainFieldIndexes_.vbatLatest = fieldIndex;
		} else if (strcmp(fieldName, "amperageLatest") == 0) {
			mainFieldIndexes_.amperageLatest = fieldIndex;
		} else if (strcmp(fieldName, "BaroAlt") == 0) {
			mainFieldIndexes_.BaroAlt = fieldIndex;
		} else if (strcmp(fieldName, "sonarRaw") == 0) {
			mainFieldIndexes_.sonarRaw = fieldIndex;
		} else if (strcmp(fieldName, "rssi") == 0) {
			mainFieldIndexes_.rssi = fieldIndex;
		} else if (strcmp(fieldName, "loopIteration") == 0) {
			mainFieldIndexes_.loopIteration = fieldIndex;
		} else if (strcmp(fieldName, "time") == 0) {
			mainFieldIndexes_.time = fieldIndex;
		}
	}
}

void Parser::identifyGPSFields(Parser::flightLogFrameDef_t *frameDef) {
	int i;

	for (i = 0; i < frameDef->fieldCount; i++) {
		const char *fieldName = frameDef->fieldName[i];

		if (strcmp(fieldName, "time") == 0) {
			gpsFieldIndexes_.time = i;
		} else if (strcmp(fieldName, "GPS_numSat") == 0) {
			gpsFieldIndexes_.GPS_numSat = i;
		} else if (strcmp(fieldName, "GPS_altitude") == 0) {
			gpsFieldIndexes_.GPS_altitude = i;
		} else if (strcmp(fieldName, "GPS_speed") == 0) {
			gpsFieldIndexes_.GPS_speed = i;
		} else if (strcmp(fieldName, "GPS_ground_course") == 0) {
			gpsFieldIndexes_.GPS_ground_course = i;
		} else if (startsWith(fieldName, "GPS_coord[")) {
			int coordIndex = atoi(fieldName + strlen("GPS_coord["));

			gpsFieldIndexes_.GPS_coord[coordIndex] = i;
		}
	}
}

void Parser::identifyGPSHomeFields(Parser::flightLogFrameDef_t *frameDef) {
	int i;

	for (i = 0; i < frameDef->fieldCount; i++) {
		const char *fieldName = frameDef->fieldName[i];

		if (strcmp(fieldName, "GPS_home[0]") == 0) {
			gpsHomeFieldIndexes_.GPS_home[0] = i;
		} else if (strcmp(fieldName, "GPS_home[1]") == 0) {
			gpsHomeFieldIndexes_.GPS_home[1] = i;
		}
	}
}

void Parser::identifySlowFields(Parser::flightLogFrameDef_t *frameDef) {
	int i;

	for (i = 0; i < frameDef->fieldCount; i++) {
		const char *fieldName = frameDef->fieldName[i];

		if (strcmp(fieldName, "flightModeFlags") == 0) {
			slowFieldIndexes_.flightModeFlags = i;
		} else if (strcmp(fieldName, "stateFlags") == 0) {
			slowFieldIndexes_.stateFlags = i;
		} else if (strcmp(fieldName, "failsafePhase") == 0) {
			slowFieldIndexes_.failsafePhase = i;
		}
	}
}

void Parser::identifyFields(uint8_t frameType, flightLogFrameDef_t *frameDef) {
	switch (frameType) {
	case 'I':
		identifyMainFields(frameDef);
		break;
	case 'G':
		identifyGPSFields(frameDef);
		break;
	case 'H':
		identifyGPSHomeFields(frameDef);
		break;
	case 'S':
		identifySlowFields(frameDef);
		break;
	default:
		;
	}
}

void Parser::parseHeaderLine() {
	char *fieldName, *fieldValue;
	const char *lineStart, *lineEnd, *separatorPos;
	int i, c;
	char valueBuffer[1024];
	union {
		float f;
		uint32_t u;
	} floatConvert;

	if (pis_.inputPeek() != ' ') {
		return;
	}

	//Skip the space
	pis_.inputTake();

	lineStart = stream->pos;
	separatorPos = 0;

	for (i = 0; i < 1024; i++) {
		c = pis_.streamReadChar();

		if (c == ':' && !separatorPos) {
			separatorPos = stream->pos - 1;
		}

		if (c == '\n')
			break;

		if (c == EOF || c == '\0')
			// Line ended before we saw a newline or it has binary stuff in there that shouldn't be there
			return;
	}

	if (!separatorPos)
		return;

	lineEnd = stream->pos;

	//Make a duplicate copy of the line so we can null-terminate the two parts
	memcpy(valueBuffer, lineStart, lineEnd - lineStart);

	fieldName = valueBuffer;
	valueBuffer[separatorPos - lineStart] = '\0';

	fieldValue = valueBuffer + (separatorPos - lineStart) + 1;
	valueBuffer[lineEnd - lineStart - 1] = '\0';

	if (startsWith(fieldName, "Field ")) {
		uint8_t frameType = (uint8_t) fieldName[strlen("Field ")];
		flightLogFrameDef_t *frameDef = &frameDefs_[frameType];

		if (endsWith(fieldName, " name")) {
			parseFieldNames(fieldValue, frameDef);
			identifyFields(frameType, frameDef);

			if (frameType == 'I') {
				// P frames are derived from I frames so copy common data over to the P frame:
				memcpy(frameDefs_['P'].fieldName, frameDef->fieldName, sizeof(frameDef->fieldName));
				frameDefs_['P'].fieldCount = frameDef->fieldCount;
			}
		} else if (endsWith(fieldName, " signed")) {
			parseCommaSeparatedIntegers(fieldValue, frameDef->fieldSigned, FLIGHT_LOG_MAX_FIELDS);

			if (frameType == 'I') {
				memcpy(frameDefs_['P'].fieldSigned, frameDef->fieldSigned, sizeof(frameDef->fieldSigned));
			}
		} else if (endsWith(fieldName, " predictor")) {
			parseCommaSeparatedIntegers(fieldValue, frameDef->predictor, FLIGHT_LOG_MAX_FIELDS);
		} else if (endsWith(fieldName, " encoding")) {
			parseCommaSeparatedIntegers(fieldValue, frameDef->encoding, FLIGHT_LOG_MAX_FIELDS);
		}
	} else if (strcmp(fieldName, "I interval") == 0) {
		frameIntervalI_ = atoi(fieldValue);
		if (frameIntervalI_ < 1)
			frameIntervalI_ = 1;
	} else if (strcmp(fieldName, "P interval") == 0) {
		char *slashPos = strchr(fieldValue, '/');

		if (slashPos) {
			frameIntervalPNum_ = atoi(fieldValue);
			frameIntervalPDenom_ = atoi(slashPos + 1);
		}
	} else if (strcmp(fieldName, "Data version") == 0) {
		dataVersion_ = atoi(fieldValue);
	} else if (strcmp(fieldName, "Firmware type") == 0) {
		if (strcmp(fieldValue, "Cleanflight") == 0)
			sysConfig_.firmwareType = FIRMWARE_TYPE_CLEANFLIGHT;
		else
			sysConfig_.firmwareType = FIRMWARE_TYPE_BASEFLIGHT;
	} else if (strcmp(fieldName, "minthrottle") == 0) {
		sysConfig_.minthrottle = atoi(fieldValue);
	} else if (strcmp(fieldName, "maxthrottle") == 0) {
		sysConfig_.maxthrottle = atoi(fieldValue);
	} else if (strcmp(fieldName, "rcRate") == 0) {
		sysConfig_.rcRate = atoi(fieldValue);
	} else if (strcmp(fieldName, "vbatscale") == 0) {
		sysConfig_.vbatscale = atoi(fieldValue);
	} else if (strcmp(fieldName, "vbatref") == 0) {
		sysConfig_.vbatref = atoi(fieldValue);
	} else if (strcmp(fieldName, "vbatcellvoltage") == 0) {
		int vbatcellvoltage[3];
		parseCommaSeparatedIntegers(fieldValue, vbatcellvoltage, 3);

		sysConfig_.vbatmincellvoltage = vbatcellvoltage[0];
		sysConfig_.vbatwarningcellvoltage = vbatcellvoltage[1];
		sysConfig_.vbatmaxcellvoltage = vbatcellvoltage[2];
	} else if (strcmp(fieldName, "currentMeter") == 0) {
		int currentMeterParams[2];

		parseCommaSeparatedIntegers(fieldValue, currentMeterParams, 2);

		sysConfig_.currentMeterOffset = currentMeterParams[0];
		sysConfig_.currentMeterScale = currentMeterParams[1];
	} else if (strcmp(fieldName, "gyro.scale") == 0) {
		floatConvert.u = strtoul(fieldValue, 0, 16);

		sysConfig_.gyroScale = floatConvert.f;

		/* Baseflight uses a gyroScale that'll give radians per microsecond as output, whereas Cleanflight produces degrees
		 * per second and leaves the conversion to radians per us to the IMU. Let's just convert Cleanflight's scale to
		 * match Baseflight so we can use Baseflight's IMU for both: */

		if (sysConfig_.firmwareType == FIRMWARE_TYPE_CLEANFLIGHT) {
			sysConfig_.gyroScale = (float) (sysConfig_.gyroScale * (M_PI / 180.0) * 0.000001);
		}
	} else if (strcmp(fieldName, "acc_1G") == 0) {
		sysConfig_.acc_1G = atoi(fieldValue);
	}
}

/**
 * Should a frame with the given index exist in this log (based on the user's selection of sampling rates)?
 */
static int shouldHaveFrame(Parser &parser, int32_t frameIndex) {
	return (frameIndex % parser.frameIntervalI_ + parser.frameIntervalPNum_ - 1) % parser.frameIntervalPDenom_ < parser.frameIntervalPNum_;
}

/**
 * Take the raw value for a a field, apply the prediction that is configured for it, and return it.
 */
static int32_t applyPrediction(Parser &parser, int fieldIndex, int fieldSigned, int predictor, uint32_t value, int32_t *current, int32_t *previous, int32_t *previous2) {

// First see if we have a prediction that doesn't require a previous frame as reference:
	switch (predictor) {
	case FLIGHT_LOG_FIELD_PREDICTOR_0:
		// No correction to apply
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE:
		value += parser.sysConfig_.minthrottle;
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_1500:
		value += 1500;
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0:
		if (parser.mainFieldIndexes_.motor[0] < 0) {
			fprintf(stderr, "Attempted to base prediction on motor[0] without that field being defined\n");
			exit(-1);
		}
		value += (uint32_t) current[parser.mainFieldIndexes_.motor[0]];
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_VBATREF:
		value += parser.sysConfig_.vbatref;
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS:
		if (!previous)
			break;

		value += (uint32_t) previous[fieldIndex];
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE:
		if (!previous)
			break;

		value += 2 * (uint32_t) previous[fieldIndex] - (uint32_t) previous2[fieldIndex];
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2:
		if (!previous)
			break;

		if (fieldSigned)
			value += (uint32_t) ((int32_t) ((uint32_t) previous[fieldIndex] + (uint32_t) previous2[fieldIndex]) / 2);
		else
			value += ((uint32_t) previous[fieldIndex] + (uint32_t) previous2[fieldIndex]) / 2;
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD:
		if (parser.gpsHomeFieldIndexes_.GPS_home[0] < 0) {
			fprintf(stderr, "Attempted to base prediction on GPS home position without GPS home frame definition\n");
			exit(-1);
		}

		value += parser.gpsHomeHistory_[1][parser.gpsHomeFieldIndexes_.GPS_home[0]];
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD_1:
		if (parser.gpsHomeFieldIndexes_.GPS_home[1] < 1) {
			fprintf(stderr, "Attempted to base prediction on GPS home position without GPS home frame definition\n");
			exit(-1);
		}

		value += parser.gpsHomeHistory_[1][parser.gpsHomeFieldIndexes_.GPS_home[1]];
		break;
	case FLIGHT_LOG_FIELD_PREDICTOR_LAST_MAIN_FRAME_TIME:
		if (parser.mainHistory_[1])
			value += parser.mainHistory_[1][FLIGHT_LOG_FIELD_INDEX_TIME];
		break;
	default:
		fprintf(stderr, "Unsupported field predictor %d\n", predictor);
		exit(-1);
	}

	return (int32_t) value;
}

/**
 * Attempt to parse the frame of the given `frameType` into the supplied `frame` buffer using the encoding/predictor
 * definitions from log->frameDefs[`frameType`].
 *
 * raw - Set to true to disable predictions (and so store raw values)
 * skippedFrames - Set to the number of field iterations that were skipped over by rate settings since the last frame.
 */
static void parseFrame(Parser &parser, uint8_t frameType, int32_t *frame, int32_t *previous, int32_t *previous2, int skippedFrames, bool raw) {
	Parser::flightLogFrameDef_t *frameDef = parser.frameDefs_[frameType];

	int *predictor = frameDef->predictor;
	int *encoding = frameDef->encoding;
	int *fieldSigned = frameDef->fieldSigned;

	int i, j, groupCount;

	i = 0;
	while (i < frameDef->fieldCount) {
		uint32_t value;
		uint32_t values[8];

		if (predictor[i] == FLIGHT_LOG_FIELD_PREDICTOR_INC) {
			frame[i] = skippedFrames + 1;

			if (previous)
				frame[i] += previous[i];

			i++;
		} else {
			switch (encoding[i]) {
			case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
				parser.pis_.streamByteAlign ();

				value = (uint32_t) parser.pis_.streamReadSignedVB();
				break;
			case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
				parser.pis_.streamByteAlign();

				value = parser.pis_.streamReadUnsignedVB();
				break;
			case FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT:
				parser.pis_.streamByteAlign();

				value = (uint32_t) -signExtend14Bit(parser.pis_.streamReadUnsignedVB());
				break;
			case FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16:
				parser.pis_.streamByteAlign();

				if (parser.dataVersion_ < 2)
					streamReadTag8_4S16_v1(parser.pis_, (int32_t*) values);
				else
					streamReadTag8_4S16_v2(parser.pis_, (int32_t*) values);

				//Apply the predictors for the fields:
				for (j = 0; j < 4; j++, i++)
					frame[i] = applyPrediction(parser, i, fieldSigned[i], raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], values[j], frame, previous, previous2);

				continue;
				break;
			case FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32:
				parser.pis_.streamByteAlign();

				streamReadTag2_3S32(parser.pis_, (int32_t*) values);

				//Apply the predictors for the fields:
				for (j = 0; j < 3; j++, i++)
					frame[i] = applyPrediction(parser, i, fieldSigned[i], raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], values[j], frame, previous, previous2);

				continue;
				break;
			case FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB:
				parser.pis_.streamByteAlign();

				//How many fields are in this encoded group? Check the subsequent field encodings:
				for (j = i + 1; j < i + 8 && j < frameDef->fieldCount; j++)
					if (encoding[j] != FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB)
						break;

				groupCount = j - i;

				streamReadTag8_8SVB(parser.pis_, (int32_t*) values, groupCount);

				for (j = 0; j < groupCount; j++, i++)
					frame[i] = applyPrediction(parser, i, fieldSigned[i], raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], values[j], frame, previous, previous2);

				continue;
				break;
			case FLIGHT_LOG_FIELD_ENCODING_ELIAS_DELTA_U32:
				value = streamReadEliasDeltaU32(parser.pis_);

				/*
				 * Reading this bitvalue may cause the stream's bit pointer to no longer lie on a byte boundary, so be sure to call
				 * streamByteAlign() if you want to read a byte from the stream later.
				 */
				break;
			case FLIGHT_LOG_FIELD_ENCODING_ELIAS_DELTA_S32:
				value = streamReadEliasDeltaS32(parser.pis_);
				break;
			case FLIGHT_LOG_FIELD_ENCODING_ELIAS_GAMMA_U32:
				value = streamReadEliasGammaU32(parser.pis_);
				break;
			case FLIGHT_LOG_FIELD_ENCODING_ELIAS_GAMMA_S32:
				value = streamReadEliasGammaS32(parser.pis_);
				break;
			case FLIGHT_LOG_FIELD_ENCODING_NULL:
				//Nothing to read
				value = 0;
				break;
			default:
				fprintf(stderr, "Unsupported field encoding %d\n", encoding[i]);
				exit(-1);
			}

			frame[i] = applyPrediction(parser, i, fieldSigned[i], raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : predictor[i], value, frame, previous, previous2);
			i++;
		}
	}

	parser.pis_.streamByteAlign();
}

/*
 * Based on the log sampling rate, work out how many frames would have been skipped after the last frame that was
 * parsed until we get to the next logged iteration.
 */
static uint32_t countIntentionallySkippedFrames(Parser &parser) {
	uint32_t count = 0, frameIndex;

	if (parser.lastMainFrameIteration_ == (uint32_t) -1) {
		// Haven't parsed a frame yet so there's no frames to skip
		return 0;
	} else {
		for (frameIndex = parser.lastMainFrameIteration_ + 1; !shouldHaveFrame(parser, frameIndex); frameIndex++) {
			count++;
		}
	}

	return count;
}

/*
 * Based on the log sampling rate, work out how many frames would have been skipped after the last frame that was
 * parsed until we get to the iteration with the given index.
 */
static uint32_t countIntentionallySkippedFramesTo(Parser &parser, uint32_t targetIteration) {
	uint32_t count = 0, frameIndex;

	if (parser.lastMainFrameIteration_ == (uint32_t) -1) {
		// Haven't parsed a frame yet so there's no frames to skip
		return 0;
	} else {
		for (frameIndex = parser.lastMainFrameIteration_ + 1; frameIndex < targetIteration; frameIndex++) {
			if (!shouldHaveFrame(parser, frameIndex)) {
				count++;
			}
		}
	}

	return count;
}

/**
 * Attempt to parse the Intraframe at the current log position into the history buffer at blackboxHistoryRing_[0].
 */
static void parseIntraframe(Parser &parser, bool raw) {
	int32_t *current = parser.mainHistory_[0];
	int32_t *previous = parser.mainHistory_[1];
	parseFrame(parser, 'I', current, previous, NULL, 0, raw);
}

/**
 * Attempt to parse the interframe at the current log position into the history buffer at mainHistory[0].
 */
static void parseInterframe(Parser &parser, bool raw) {
	int32_t *current = parser.mainHistory_[0];
	int32_t *previous = parser.mainHistory_[1];
	int32_t *previous2 = parser.mainHistory_[2];

	parser.lastSkippedFrames_ = countIntentionallySkippedFrames(parser);

	parseFrame(parser, 'P', current, previous, previous2, parser.lastSkippedFrames_, raw);
}

static void parseGPSFrame(Parser &parser, bool raw) {
	parseFrame(parser, 'G', parser.lastGPS_, NULL, NULL, 0, raw);
}

static void parseGPSHomeFrame(Parser &parser, bool raw) {
	parseFrame(parser, 'H', parser.gpsHomeHistory_[0], NULL, NULL, 0, raw);
}

static void parseSlowFrame(Parser &parser, bool raw) {
	parseFrame(parser, 'S', parser.lastSlow_, NULL, NULL, 0, raw);
}

/**
 * Attempt to parse an event frame at the current location into the log->prvt->lastEvent struct.
 * Return false if the event couldn't be parsed (e.g. unknown event ID), or true if it might have been
 * parsed successfully.
 */
static void parseEventFrame(Parser &parser, bool raw) {
	static const char END_OF_LOG_MESSAGE[] = "End of log\0";
	enum {
		END_OF_LOG_MESSAGE_LEN = 11
	};

	char endMessage[END_OF_LOG_MESSAGE_LEN];
	(void) raw;
	uint8_t eventType = parser.pis_.streamReadByte();

	flightLogEventData_t *data = &parser.lastEvent_.data;
	parser.lastEvent_.event = eventType;

	switch (eventType) {
	case FLIGHT_LOG_EVENT_SYNC_BEEP:
		data->syncBeep.time = parser.pis_.streamReadUnsignedVB();
		break;
	case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START:
		data->autotuneCycleStart.phase = parser.pis_.streamReadByte();
		data->autotuneCycleStart.cycle = parser.pis_.streamReadByte();
		data->autotuneCycleStart.p = parser.pis_.streamReadByte();
		data->autotuneCycleStart.i = parser.pis_.streamReadByte();
		data->autotuneCycleStart.d = parser.pis_.streamReadByte();
		break;
	case FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT:
		data->autotuneCycleResult.flags = parser.pis_.streamReadByte();
		data->autotuneCycleResult.p = parser.pis_.streamReadByte();
		data->autotuneCycleResult.i = parser.pis_.streamReadByte();
		data->autotuneCycleResult.d = parser.pis_.streamReadByte();
		break;
	case FLIGHT_LOG_EVENT_AUTOTUNE_TARGETS:
		data->autotuneTargets.currentAngle = streamReadS16(parser.pis_);
		data->autotuneTargets.targetAngle = (int8_t) parser.pis_.streamReadByte();
		data->autotuneTargets.targetAngleAtPeak = (int8_t) parser.pis_.streamReadByte();
		data->autotuneTargets.firstPeakAngle = streamReadS16(parser.pis_);
		data->autotuneTargets.secondPeakAngle = streamReadS16(parser.pis_);
		break;
	case FLIGHT_LOG_EVENT_GTUNE_CYCLE_RESULT:
		data->gtuneCycleResult.axis = parser.pis_.streamReadByte();
		data->gtuneCycleResult.gyroAVG = parser.pis_.streamReadSignedVB();
		data->gtuneCycleResult.newP = streamReadS16(parser.pis_);
		break;
	case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
		data->inflightAdjustment.adjustmentFunction = parser.pis_.streamReadByte();
		if (data->inflightAdjustment.adjustmentFunction > 127) {
			data->inflightAdjustment.newFloatValue = streamReadRawFloat(parser.pis_);
		} else {
			data->inflightAdjustment.newValue = parser.pis_.streamReadSignedVB();
		}
		break;
	case FLIGHT_LOG_EVENT_LOGGING_RESUME:
		data->loggingResume.logIteration = parser.pis_.streamReadUnsignedVB();
		data->loggingResume.currentTime = parser.pis_.streamReadUnsignedVB();
		break;
	case FLIGHT_LOG_EVENT_LOG_END:
		parser.pis_.streamRead(endMessage, END_OF_LOG_MESSAGE_LEN);

		if (strncmp(endMessage, END_OF_LOG_MESSAGE, END_OF_LOG_MESSAGE_LEN) == 0) {
//Adjust the end of stream so we stop reading, this log is done
			stream->end = stream->pos;
		} else {
			/*
			 * This isn't the real end of log message, it's probably just some bytes that happened to look like
			 * an event header.
			 */
			parser.lastEvent_.event = -1;
		}
		break;
	default:
		parser.lastEvent_.event = -1;
	}
}

static void updateMainFieldStatistics(Parser &parser, int32_t *fields) {
	int i;
	Parser::flightLogFrameDef_t *frameDef = &parser.frameDefs_['I'];

	if (!parser.stats_.haveFieldStats) {
		//If this is the first frame, there are no minimums or maximums in the stats to compare with
		for (i = 0; i < frameDef->fieldCount; i++) {
			if (frameDef->fieldSigned[i]) {
				parser.stats_.field[i].max = fields[i];
				parser.stats_.field[i].min = fields[i];
			} else {
				parser.stats_.field[i].max = (uint32_t) fields[i];
				parser.stats_.field[i].min = (uint32_t) fields[i];
			}
		}

		parser.stats_.haveFieldStats = true;
	} else {
		for (i = 0; i < frameDef->fieldCount; i++) {
			if (frameDef->fieldSigned[i]) {
				parser.stats_.field[i].max = fields[i] > parser.stats_.field[i].max ? fields[i] : parser.stats_.field[i].max;
				parser.stats_.field[i].min = fields[i] < parser.stats_.field[i].min ? fields[i] : parser.stats_.field[i].min;
			} else {
				parser.stats_.field[i].max = (uint32_t) fields[i] > parser.stats_.field[i].max ? (uint32_t) fields[i] : parser.stats_.field[i].max;
				parser.stats_.field[i].min = (uint32_t) fields[i] < parser.stats_.field[i].min ? (uint32_t) fields[i] : parser.stats_.field[i].min;
			}
		}
	}
}

#define ADCVREF 33L

unsigned int Parser::flightLogVbatADCToMillivolts(uint16_t vbatADC) {
	// ADC is 12 bit (i.e. max 0xFFF), voltage reference is 3.3V, vbatscale is premultiplied by 100
	return (vbatADC * ADCVREF * 10 * sysConfig_.vbatscale) / 0xFFF;
}

unsigned int Parser::flightLogAmperageADCToMilliamps(uint16_t amperageADC) {
	int32_t millivolts;

	millivolts = ((uint32_t) amperageADC * ADCVREF * 100) / 4095;
	millivolts -= sysConfig_.currentMeterOffset;

	return ((int64_t) millivolts * 10000) / sysConfig_.currentMeterScale;
}

int Parser::flightLogEstimateNumCells() {
	int i;
	int refVoltage;

	refVoltage = flightLogVbatADCToMillivolts(sysConfig_.vbatref) / 100;

	for (i = 1; i < 8; i++) {
		if (refVoltage < i * sysConfig_.vbatmaxcellvoltage)
			break;
	}

	return i;
}

double Parser::flightlogAccelerationRawToGs(int32_t accRaw) {
	return (double) accRaw / sysConfig_.acc_1G;
}

double Parser::flightlogGyroToRadiansPerSecond(int32_t gyroRaw) {
	// gyroScale is set to give radians per microsecond, so multiply by 1,000,000 out to get the per-second value
	return (double) sysConfig_.gyroScale * 1000000 * gyroRaw;
}

static void flightlogDecodeFlagsToString(uint32_t flags, int numFlags, const char * const *flagNames, char *dest, unsigned destLen) {
	bool printedFlag = false;
	const char NO_FLAGS_MESSAGE[] = "0";

	// The buffer should at least be large enough for us to add the "no flags present" message in!
	if (destLen < strlen(NO_FLAGS_MESSAGE) + 1) {
		fprintf(stderr, "Flag buffer too short\n");
		exit(-1);
	}

	for (int i = 0; i < numFlags; i++) {
		if ((flags & (1 << i)) != 0) {
			const char *flagName = flagNames[i];
			unsigned flagNameLen = strlen(flagName);

			if (destLen < (printedFlag ? 1 : 0) + flagNameLen + 1 /* Null-terminator */) {
// Not enough room in the dest string to fit this flag
				fprintf(stderr, "Flag buffer too short\n");
				exit(-1);
			}

			if (printedFlag) {
				*dest = '|';
				dest++;
				destLen--;
			} else {
				printedFlag = true;
			}

			strcpy(dest, flagName);

			dest += flagNameLen;
			destLen -= flagNameLen;
		}
	}

	if (!printedFlag) {
		strcpy(dest, NO_FLAGS_MESSAGE);
	}
}

void flightlogDecodeEnumToString(uint32_t value, unsigned numEnums, const char * const *enumNames, char *dest, unsigned destLen) {
	assert(destLen > 1);

	if (value < numEnums) {
		const char *name = enumNames[value];

		if (strlen(name) < destLen) {
			strcpy(dest, name);
		} else {
			dest[0] = '\0';
		}
	} else {
		// Since we don't have a name for this value, print it as a raw integer instead

		snprintf(dest, destLen, "%u", value);
	}
}

void Parser::flightlogFlightModeToString(uint32_t flightMode, char *dest, int destLen) {
	flightlogDecodeFlagsToString(flightMode, FLIGHT_LOG_FLIGHT_MODE_COUNT, FLIGHT_LOG_FLIGHT_MODE_NAME, dest, destLen);
}

void Parser::flightlogFlightStateToString(uint32_t flightState, char *dest, int destLen) {
	flightlogDecodeFlagsToString(flightState, FLIGHT_LOG_FLIGHT_STATE_COUNT, FLIGHT_LOG_FLIGHT_STATE_NAME, dest, destLen);
}

void Parser::flightlogFailsafePhaseToString(uint8_t failsafePhase, char *dest, int destLen) {
	flightlogDecodeEnumToString(failsafePhase, FLIGHT_LOG_FAILSAFE_PHASE_COUNT, FLIGHT_LOG_FAILSAFE_PHASE_NAME, dest, destLen);
}

Parser::flightLogFrameType_t* Parser::getFrameType(uint8_t c) {
	for (int i = 0; i < (int) ARRAY_LENGTH(frameTypes_); i++)
		if (frameTypes_[i].marker == c)
			return &frameTypes_[i];

	return 0;
}

static void flightLoginvalidateStream(Parser &parser) {
	parser.mainStreamIsValid_ = false;
	parser.mainHistory_[1] = 0;
	parser.mainHistory_[2] = 0;
}

static bool completeIntraframe(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw) {
	bool acceptFrame = true;

	// Do we have a previous frame to use as a reference to validate field values against?
	if (!raw && parser.lastMainFrameIteration_ != (uint32_t) -1) {
		/*
		 * Check that iteration count and time didn't move backwards, and didn't move forward too much.
		 */
		acceptFrame = (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_ITERATION]
				>= parser.lastMainFrameIteration_&& (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_ITERATION] < parser.lastMainFrameIteration_ + MAXIMUM_ITERATION_JUMP_BETWEEN_FRAMES
				&& (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_TIME] >= parser.lastMainFrameTime_
				&& (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_TIME] < parser.lastMainFrameTime_ + MAXIMUM_TIME_JUMP_BETWEEN_FRAMES;
	}

	if (acceptFrame) {
		parser.stats_.intentionallyAbsentIterations += countIntentionallySkippedFramesTo(parser, (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_ITERATION]);

		parser.lastMainFrameIteration_ = (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_ITERATION];
		parser.lastMainFrameTime_ = (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_TIME];

		parser.mainStreamIsValid_ = true;

		updateMainFieldStatistics(parser, parser.mainHistory_[0]);
	} else {
		flightLoginvalidateStream(parser);
	}

	if (parser.onFrameReady)
		parser.onFrameReady(parser, parser.mainStreamIsValid_, parser.mainHistory_[0], frameType, parser.frameDefs_[(int) frameType].fieldCount, frameStart - stream->data, frameEnd - frameStart);

	if (acceptFrame) {
		// Rotate history buffers

// Both the previous and previous-previous states become the I-frame, because we can't look further into the past than the I-frame
		parser.mainHistory_[1] = parser.mainHistory_[0];
		parser.mainHistory_[2] = parser.mainHistory_[0];

// And advance the current frame into an empty space ready to be filled
		parser.mainHistory_[0] += FLIGHT_LOG_MAX_FIELDS;
		if (parser.mainHistory_[0] >= &parser.blackboxHistoryRing_[3][0])
			parser.mainHistory_[0] = &parser.blackboxHistoryRing_[0][0];
	}

	return acceptFrame;
}

static bool completeInterframe(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw) {
	(void) frameType;
	(void) raw;

	// Reject this frame if the time or iteration count jumped too far
	if (parser.mainStreamIsValid_ && !raw
			&& ((uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_TIME] > parser.lastMainFrameTime_ + MAXIMUM_TIME_JUMP_BETWEEN_FRAMES
					|| (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_ITERATION] > parser.lastMainFrameIteration_ + MAXIMUM_ITERATION_JUMP_BETWEEN_FRAMES)) {
		parser.mainStreamIsValid_ = false;
	}

	if (parser.mainStreamIsValid_) {
		parser.lastMainFrameIteration_ = (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_ITERATION];
		parser.lastMainFrameTime_ = (uint32_t) parser.mainHistory_[0][FLIGHT_LOG_FIELD_INDEX_TIME];

		parser.stats_.intentionallyAbsentIterations += parser.lastSkippedFrames_;

		updateMainFieldStatistics(parser, parser.mainHistory_[0]);
	}

	//Receiving a P frame can't resynchronise the stream so it doesn't set mainStreamIsValid to true

	if (parser.onFrameReady)
		parser.onFrameReady(log, parser.mainStreamIsValid_, parser.mainHistory_[0], frameType, parser.frameDefs_['I'].fieldCount, frameStart - stream->data, frameEnd - frameStart);

	if (parser.mainStreamIsValid_) {
		// Rotate history buffers
		parser.mainHistory_[2] = parser.mainHistory_[1];
		parser.mainHistory_[1] = parser.mainHistory_[0];

// And advance the current frame into an empty space ready to be filled
		parser.mainHistory_[0] += FLIGHT_LOG_MAX_FIELDS;
		if (parser.mainHistory_[0] >= &parser.blackboxHistoryRing_[3][0])
			parser.mainHistory_[0] = &parser.blackboxHistoryRing_[0][0];
	}

	return parser.mainStreamIsValid_;
}

static bool completeEventFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw) {
	flightLogEvent_t *lastEvent = &parser.lastEvent_;

	(void) frameType;
	(void) frameStart;
	(void) frameEnd;
	(void) raw;

	//Don't bother reporting invalid event types since they're likely just garbage data that happened to look like an event
	if (lastEvent->event != (FlightLogEvent) -1) {
		switch (lastEvent->event) {
		case FLIGHT_LOG_EVENT_LOGGING_RESUME:
			/*
			 * Bring the "last time" and "last iteration" up to the new resume time so we accept the sudden jump into
			 * the future.
			 */
			parser.lastMainFrameIteration_ = lastEvent->data.loggingResume.logIteration;
			parser.lastMainFrameTime_ = lastEvent->data.loggingResume.currentTime;
			break;
		default:
			;
		}

		if (parser.onEvent) {
			parser.onEvent(log, lastEvent);
		}

		return true;
	}

	return false;
}

static bool completeGPSHomeFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw) {
	(void) frameType;
	(void) frameStart;
	(void) frameEnd;
	(void) raw;

	//Copy the decoded frame into the "last state" entry of gpsHomeHistory to publish it:
	memcpy(&parser.gpsHomeHistory_[1], &parser.gpsHomeHistory_[0], sizeof(*parser.gpsHomeHistory_));
	parser.gpsHomeIsValid_ = true;

	if (parser.onFrameReady) {
		parser.onFrameReady(parser, true, parser.gpsHomeHistory_[1], frameType, parser.frameDefs_[frameType].fieldCount, frameStart - stream->data, frameEnd - frameStart);
	}

	return true;
}

static bool completeGPSFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw) {
	(void) frameType;
	(void) frameStart;
	(void) frameEnd;
	(void) raw;

	if (parser.onFrameReady) {
		parser.onFrameReady(log, parser.gpsHomeIsValid_, parser.lastGPS_, frameType, parser.frameDefs_[frameType].fieldCount, frameStart - stream->data, frameEnd - frameStart);
	}

	return true;
}

static bool completeSlowFrame(Parser &parser, uint8_t frameType, const char *frameStart, const char *frameEnd, bool raw) {
	(void) frameType;
	(void) frameStart;
	(void) frameEnd;
	(void) raw;

	if (parser.onFrameReady) {
		parser.onFrameReady(log, true, parser.lastSlow_, frameType, parser.frameDefs_[frameType].fieldCount, frameStart - stream->data, frameEnd - frameStart);
	}

	return true;
}

bool Parser::parse(bool raw) {
	char *frameStart;
	flightLogFrameType_t *frameType;
	flightLogFrameType_t *lastFrameType;

	ParserState parserState = PARSER_STATE_HEADER;

	while (1) {
		const char command = pis_.inputPeek();

		switch (parserState) {
		case PARSER_STATE_HEADER:
			switch (command) {
			case 'H':
				pis_.inputTake();
				parseHeaderLine();
				break;
			case EOF:
				fprintf(stderr, "Data file contained no events\n");
				return false;
			default:
				frameType = getFrameType(command);

				if (frameType) {
					if (frameDefs_['I'].fieldCount == 0) {
						fprintf(stderr, "Data file is missing field name definitions\n");
						return false;
					}

					/* Home coord predictors appear in pairs (lat/lon), but the predictor ID is the same for both. It's easier to
					 * apply the right predictor during parsing if we rewrite the predictor ID for the second half of the pair here:
					 */
					for (int i = 1; i < frameDefs_['G'].fieldCount; i++) {
						if (frameDefs_['G'].predictor[i - 1] == FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD && frameDefs_['G'].predictor[i] == FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD) {
							frameDefs_['G'].predictor[i] = FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD_1;
						}
					}

					parserState = PARSER_STATE_DATA;
					lastFrameTypemainHistory_[L;
					frameStart = prvt->stream->pos;

					if (onMetadataReady)
						onMetadataReady(log);
				} // else skip garbage which apparently precedes the first data frame
				break;
			}
			break;
		case PARSER_STATE_DATA:
			if (lastFrameType) {
				// Is this the beginning of a new frame?
				frameType = command == EOF ? 0 : getFrameType((uint8_t) command);
				looksLikeFrameCompleted_ = frameType || (!prematureEof_ && command == EOF);

				// If we see what looks like the beginning of a new frame, assume that the previous frame was valid:
				if (lastFrameSize <= FLIGHT_LOG_MAX_FRAME_LENGTH && looksLikeFrameCompleted_) {
					bool frameAccepted = true;

					if (lastFrameType->complete)
						frameAccepted = lastFrameType->complete(log, pis, lastFrameType->marker, frameStart, frameEnd, raw);

					if (frameAccepted) {
						//Update statistics for this frame type
						stats_.frame[lastFrameType->marker].bytes += lastFrameSize;
						stats_.frame[lastFrameType->marker].sizeCount[lastFrameSize]++;
						stats_.frame[lastFrameType->marker].validCount++;
					} else {
						stats_.frame[lastFrameType->marker].desyncCount++;
					}
				} else {
					//The previous frame was corrupt

//We need to resynchronise before we can deliver another main frame:
					mainStreamIsValid_ = false;
					stats_.frame[lastFrameType->marker].corruptCount++;
					stats_.totalCorruptFrames++;

					//Let the caller know there was a corrupt frame (don't give them a pointer to the frame data because it is totally worthless)
					if (onFrameReady)
						onFrameReady(log, false, 0, lastFrameType->marker, 0, frameStart - prvt->stream->data, lastFrameSize);

					/*
					 * Start the search for a frame beginning after the first byte of the previous corrupt frame.
					 * This way we can find the start of the next frame after the corrupt frame if the corrupt frame
					 * was truncated.
					 */
					prvt->stream->pos = frameStart + 1;
					lastFrameType = NULL;
					prematureEof_ = false;
					prvt->stream->eof = false;
					continue;
				}
			}

			if (command == EOF)
				goto done;

			frameType = getFrameType((uint8_t) command);
			frameStart = prvt->stream->pos - 1;

			if (frameType) {
				frameType->parse(log, pis, raw);
			} else {
				prvt->mainStreamIsValid = false;
			}

			//We shouldn't read an EOF during reading a frame (that'd imply the frame was truncated)
			if (prvt->stream->eof)
				prematureEof_ = true;

			lastFrameType = frameType;
			break;
		}
	}
	done: ;

	return true;
}

}
