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

DT_transformation KIN_getTransMat(const DT_leg* const);
void KIN_calcDH(const DT_leg* const, DT_double**);
void KIN_calcServos(const DT_point* const, DT_leg* const);
DT_point KIN_calcLocalPoint(const DT_point* const, const DT_transformation* const);
DT_bool KIN_makeMovement(DT_leg* leg_l, DT_leg* leg_r);
#endif /* KINEMATICS_H_ */
