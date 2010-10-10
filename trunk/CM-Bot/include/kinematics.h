/*
 * Solution of kinematic and inverse kinematic problem.
 *
 * Created on: 27.09.2010
 * Author: christof
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "utils.h"

#define KIN_ROWS 4
#define KIN_COLUMNS 4

void KIN_calculateDH(const servos servos, double** destinationDh03);

servos KIN_calculateServos(const point point);

#endif /* KINEMATICS_H_ */
