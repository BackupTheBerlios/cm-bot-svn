/**
 * \file	utils.h
 *
 * \brief	TODO
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "datatypes.h"

#define UTL_DEG 1
#define UTL_RAD 0

#define DEBUG_ON
#ifdef DEBUG_ON
	#define DEBUG(output) UTL_printDebug output;
	#define DEBUG_BYTE(output) UTL_printDebugByte output;
#else
	#define DEBUG(output) /* no debug */
	#define DEBUG_BYTE(output) /* no debug */
#endif

void UTL_printMatrix(DT_double**, DT_size, DT_size);
void UTL_printServos(DT_servos, DT_type);
void UTL_printPoint(DT_point);

DT_double UTL_getRadiant(DT_double);
DT_double UTL_getDegree(DT_double);
DT_point UTL_getPointOfDH(DT_double**);

void UTL_printDebug(DT_char*, DT_size);
void UTL_printDebugByte(DT_byte*, DT_size);
DT_byte UTL_byteToHexChar(DT_char*, DT_byte*, DT_size);

void UTL_wait(DT_size);

#endif /* UTILS_H_ */
