/**
 * \file	dynamixel.h
 *
 * \brief	TODO
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "datatypes.h"

#define DNX_BRDCAST_ID 0xFE

DT_byte DNX_getChecksum(DT_byte*, DT_size);
void DNX_setAngle(DT_byte, DT_double);
void DNX_setId(DT_byte, DT_byte);
void DNX_setSpeed(DT_byte, DT_byte);
void DNX_setLed(DT_byte, DT_byte);

DT_double DNX_getAngle(DT_byte);
DT_byte DNX_getSpeed(DT_byte);
DT_byte DNX_getLed(DT_byte);

#endif /* DYNAMIXEL_H_ */
