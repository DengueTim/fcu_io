#ifndef DECODERS_H_
#define DECODERS_H_

/* Decoders for reading advanced data formats from data streams */
#include <stdint.h>

#include "parser_input_stream.h"

void streamReadTag2_3S32(blackbox::ParserInputStream &pis, int32_t *values);
void streamReadTag8_4S16_v1(blackbox::ParserInputStream &pis, int32_t *values);
void streamReadTag8_4S16_v2(blackbox::ParserInputStream &pis, int32_t *values);
void streamReadTag8_8SVB(blackbox::ParserInputStream &pis, int32_t *values, int valueCount);

int16_t streamReadS16(blackbox::ParserInputStream &pis);

float streamReadRawFloat(blackbox::ParserInputStream &pis);

uint32_t streamReadEliasDeltaU32(blackbox::ParserInputStream &pis);
int32_t streamReadEliasDeltaS32(blackbox::ParserInputStream &pis);

uint32_t streamReadEliasGammaU32(blackbox::ParserInputStream &pis);
int32_t streamReadEliasGammaS32(blackbox::ParserInputStream &pis);

#endif
