/**
 * \file	kinematics.h
 *
 * \brief	TODO
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "datatypes.h"

#define KIN_ROWS 4
#define KIN_COLUMNS 4

void KIN_calculateDH(const DT_leg, DT_double**);

DT_leg KIN_calculateServos(const DT_point);

#endif /* KINEMATICS_H_ */
