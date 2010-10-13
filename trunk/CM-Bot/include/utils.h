/*
 * Methods to convert or print data.
 *
 * Created on: 27.09.2010
 * Author: christof
 */

#include <stdint.h>

#ifndef UTILS_H_
#define UTILS_H_

#define UTL_DEG 1
#define UTL_RAD 0

#define DEBUG_ON
#ifdef DEBUG_ON
	// #include <stdio.h>
	#define DEBUG(output) UTL_printDebug output;
	#define DEBUG_BYTE(output) UTL_printDebugByte output;
#else
	#define DEBUG(output) /* no debug */
	#define DEBUG_BYTE(output) /* no debug */
#endif

struct Servos {
	double v1, v2, v3;
};

struct Point {
	double x, y, z;
};

typedef struct Point point;
typedef struct Servos servos;
typedef uint8_t byte;

void UTL_printMatrix(double**, int, int);
void UTL_printServos(servos, char);
void UTL_printPoint(point);

double UTL_getRadiant(double);
double UTL_getDegree(double );
point UTL_getPointOfDH(double**);

void UTL_printDebug(char*, byte);
void UTL_printDebugByte(byte*, byte);
byte UTL_byteToHexChar(char*, byte*, byte);

void UTL_wait(byte);

#endif /* UTILS_H_ */
