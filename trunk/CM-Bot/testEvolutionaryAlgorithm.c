/**
 * \file	testCom.c
 *
 * \brief	Testprogramm für Evolutionären Algorithmus zur Startpunktfindung.
 */

#define TEST_ON
#ifdef TEST_ON

#include <stdio.h>
#include <math.h>
#include "include/kinematics.h"
#include "include/utils.h"
#include "include/evolutionaryHelper.h"
#include "include/evolutionaryAlgorithm.h"
#include "include/dynamixel.h"
#include "include/xmega.h"
#include "include/communication.h"
#include "include/movement.h"
#include "include/remote.h"

DT_leg leg_r, leg_l;
DT_byte cpuID;

void master();

int main(void) {
	XM_init_cpu();
	XM_init_dnx();
	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	XM_init_com(cpuID);

	XM_init_remote();

	XM_LED_OFF

	switch (cpuID) {
	case COM_MASTER:
		XM_init_remote();
		DEBUG(("Master",sizeof("Master")))
		master();
		break;
	case COM_SLAVE1B:
		DEBUG(("Slave1",sizeof("Slave1")))
		//MV_slave(cpuID, &leg_r, &leg_l);
		break;
	case COM_SLAVE3F:
		DEBUG(("Slave3",sizeof("Slave3")))
		//MV_slave(cpuID, &leg_r, &leg_l);
		break;
	default: //case NOCPUID:
		DEBUG (("NoCpuID",sizeof("NoCpuID")))
		XM_LED_OFF
		break;
	}
	return 0;
}

void invertVector(DT_vector * v) {
	v->x = -v->x;
	v->y = -v->y;
}

void master() {
	DT_vector v;
	DT_individuum A, B;
	DT_point Startpoint;
	DT_cmd cmd;
	DT_byte config;

	while (1) {
		// Lokaler Vektor
		v.x = 0;
		v.y = 0;
		do {
			cmd = RMT_getCommand();
			if (RMT_isUpPressed(cmd))
				v.y += 10;
			if (RMT_isDownPressed(cmd))
				v.y -= 10;
			if (RMT_isLeftPressed(cmd))
				v.x += 10;
			if (RMT_isRightPressed(cmd))
				v.x -= 10;
		} while (!RMT_isButton6Pressed(cmd));

		A = evolutionaryAlgorithm(10, 5, &v);
		Startpoint = getPointFromIndividuum(&A);
		MV_point(&leg_l, &Startpoint, false);
		Startpoint.z += 100;
		MV_point(&leg_r, &Startpoint, false);
		invertVector(&v);
		B = evolutionaryAlgorithm(10, 5, &v);
		Startpoint = getPointFromIndividuum(&B);
		config = COM_CONF_RIGHT;
		COM_sendPoint(COM_SLAVE1B, &Startpoint, config);
		COM_sendPoint(COM_SLAVE3F, &Startpoint, config);
		Startpoint.z += 100;
		config = COM_CONF_LEFT;
		COM_sendPoint(COM_SLAVE1B, &Startpoint, config);
		COM_sendPoint(COM_SLAVE3F, &Startpoint, config);


		COM_sendAction(COM_BRDCAST_ID);
		MV_action(&leg_r, &leg_l);
	}
}

#endif
