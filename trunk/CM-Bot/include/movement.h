/*
 * movement.h
 *
 *  Created on: 23.11.2010
 *      Author: christof
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "datatypes.h"

void MV_action(DT_leg* const, DT_leg* const);
void MV_slave(DT_byte, DT_leg* const, DT_leg* const);
void MV_slaveStatus(const DT_byte* const, const DT_size);
void MV_slavePoint(DT_leg* const, DT_leg* const, const DT_byte* const, DT_size);
void MV_point(DT_leg* const, const DT_point* const, DT_bool);

#endif /* MOVEMENT_H_ */
