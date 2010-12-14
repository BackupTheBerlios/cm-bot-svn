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
DT_byte MasterActive, SlavesActive, MasterInactive, SlavesInactive;
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

void switchLegs() {
	if (MasterActive == COM_CONF_LEFT) {
		MasterActive = COM_CONF_RIGHT;
		SlavesActive = COM_CONF_LEFT;
		MasterInactive = COM_CONF_LEFT;
		SlavesInactive = COM_CONF_RIGHT;
	} else {
		MasterActive = COM_CONF_LEFT;
		SlavesActive = COM_CONF_RIGHT;
		MasterInactive = COM_CONF_RIGHT;
		SlavesInactive = COM_CONF_LEFT;
	}
}

void initConf() {
	MasterActive = COM_CONF_LEFT;
	SlavesActive = COM_CONF_RIGHT;
	MasterInactive = COM_CONF_RIGHT;
	SlavesInactive = COM_CONF_LEFT;
}

void TripodGaitMove(DT_point* pM, DT_point* pS, DT_double offset) {
	DT_double z;
	z = pM->z;
	if (MasterActive == COM_CONF_LEFT) {
		MV_point(&leg_l, pM, false);
		pM->z += offset;
		MV_point(&leg_r, pM, false);
	} else {
		MV_point(&leg_r, pM, false);
		pM->z += offset;
		MV_point(&leg_l, pM, false);
	}
	pM->z = z;
	z = pS->z;
	COM_sendPoint(COM_SLAVE1B, pS, SlavesActive);
	COM_sendPoint(COM_SLAVE3F, pS, SlavesActive);
	pS->z += offset;
	COM_sendPoint(COM_SLAVE1B, pS, SlavesInactive);
	COM_sendPoint(COM_SLAVE3F, pS, SlavesInactive);
	pS->z = z;
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

DT_point pM, pS;

void init_pMpS(){
	pM.x = 110.1041;
	pM.y = 0;
	pM.z = Z;
	pS.x = 110.1041;
	pS.y = 0;
	pS.z = Z;
}

void EvolutionaryTripodGaitMove(DT_vector * v) {
	DT_individuum A, B;
	A = evolutionaryAlgorithm(10, 5, v);
	pM = getPointFromIndividuum(&A);
	invertVector(v);
	B = evolutionaryAlgorithm(10, 5, v);
	pS = getPointFromIndividuum(&B);
	TripodGaitMove(&pM, &pS, 50);
}

void prepareStepMove(DT_point* pM, DT_point* pS, DT_double offset) {
	DT_double z;
	z = pM->z;
	if (MasterInactive == COM_CONF_LEFT) {
		pM->z += offset;
		MV_point(&leg_l, pM, false);
	} else {
		pM->z += offset;
		MV_point(&leg_r, pM, false);
	}
	pM->z = z;
	z = pS->z;
	pS->z += offset;
	COM_sendPoint(COM_SLAVE1B, pS, SlavesInactive);
	COM_sendPoint(COM_SLAVE3F, pS, SlavesInactive);
	pS->z = z;
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}


void evolutionaryPrepareStepMove(DT_vector * v){
	DT_individuum A, B;
	A = evolutionaryAlgorithm(10, 5, v);
	pM = getPointFromIndividuum(&A);
	invertVector(v);
	B = evolutionaryAlgorithm(10, 5, v);
	pS = getPointFromIndividuum(&B);
	prepareStepMove(&pM, &pS, 50);
}

void master() {
	DT_cmd cmd;
	DT_vector v;

	init_pMpS();
	initConf();

	TripodGaitMove(&pM, &pS, 0);

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
			if (RMT_isButton1Pressed(cmd))
				TripodGaitMove(&pM, &pS, 0);
			if (RMT_isButton2Pressed(cmd))
				switchLegs();
		} while (!RMT_isButton6Pressed(cmd));

		evolutionaryPrepareStepMove(&v);
		prepareStepMove(&pM, &pS, 0);

//		EvolutionaryTripodGaitMove(&v);

		// 1. Alle Beine auf dem Boden
		// 2. Inaktive Beine in die Luft
		// 3. Inaktive Beine fahren Startpunkt an
		// 4. Alle Beine auf dem Boden
		// 5. Beine Wechseln
		// 6. Inaktive Beine in die Luft
		// 7. Aktive Beine führen Bewegung aus

	}
}

#endif
