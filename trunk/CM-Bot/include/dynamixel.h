/**
 * \file	dynamixel.h
 *
 * \brief	TODO
 */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include "datatypes.h"

#define DNX_BRDCAST_ID 0xFE

DT_byte DNX_getChecksum(const DT_byte* const, DT_size);
void DNX_setAngle(DT_byte, DT_double);
void DNX_setId(DT_byte, DT_byte);
void DNX_setSpeed(DT_byte, DT_byte);
void DNX_setLed(DT_byte, DT_byte);

DT_double DNX_getAngle(DT_byte);
DT_byte DNX_getSpeed(DT_byte);
DT_byte DNX_getLed(DT_byte);
DT_byte DNX_getConnectedIDs();
DT_leg leg_r, leg_l;

#endif /* DYNAMIXEL_H_ */
