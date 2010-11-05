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

void KIN_calculateDH(const DT_leg* const, DT_double**);

void KIN_calculateServos(const DT_point* const, DT_leg* const);

#endif /* KINEMATICS_H_ */