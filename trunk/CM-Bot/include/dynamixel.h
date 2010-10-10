/*
 * Methods to control the dynamixel.
 *
 * Created on: 27.09.2010
 * Author: christof
 */

#include "utils.h"

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#define DNX_BRDCAST_ID 0xFE

byte DNX_getChecksum(byte*, byte);
void DNX_setAngle(byte, byte);
void DNX_setId(byte, byte);
void DNX_setSpeed(byte, byte);
void DNX_setLed(byte, byte);
void DNX_sendTest();

double DNX_getAngle(byte);
byte DNX_getSpeed(byte);
byte DNX_getLed(byte);

#endif /* DYNAMIXEL_H_ */
