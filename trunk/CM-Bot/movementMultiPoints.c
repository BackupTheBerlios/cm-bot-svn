/**
 * \file movementMultiPoints.c
 *
 *  \brief	Algorithmus fuer das Vorwaertslaufen ueber 4 Punkte.
 */

#define TEST_ON TEST
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/communication.h"
#include "include/movement.h"
#include "include/remote.h"

DT_leg leg_r, leg_l;
DT_byte cpuID;

#define STEP_SIZE	3

DT_point ma_calcStartPoint(const DT_point* const v, DT_double z,
		const DT_byte side) {
	// TODO Startpunkt für vektor im Arbeitsraum suchen
	/*
	 * - Arbeitsraum als Kugel erstellen
	 * - Kreis/Fläche aus Kugel und geg. z errechnen -> Arbeitsfläche eingrenzen -> Kreisegment
	 * - In definierter Abeitsfläche (Kreissegment) längste Sekante mit dem Vektor v suchen/berechnen und deren Schnittpunkte -> Startpunkt
	 * - wie neues z fuer Punkt berechnen, da z Abhänhig von x und y
	 */
	DT_point p;
	return p;
}

DT_point ma_movePnt(DT_point* const p, const DT_point* const v) {
	// TODO	wie neues z fuer Punkt berechnen, da z Abhänhig von x und y
	DT_point pMv;
	pMv.x = p->x + v->x;
	pMv.y = p->y + v->y;
	pMv.z = p->z + v->z;
	return pMv;
}

DT_point ma_calcUnitVec(const DT_point* const v) {
	// TODO Vektor auf gewuensche Schrittweite umrechnennach der
	DT_point vUnt;
	return vUnt;
}

DT_point ma_negateVec(const DT_point* const v) {
	DT_point vNeg;
	vNeg.x = -v->x;
	vNeg.y = -v->y;
	vNeg.z = -v->z;
	return vNeg;
}

void ma_switchLegs(DT_byte* side, DT_leg* leg_dwn, DT_leg* leg_up,
		DT_byte* slave_dwn, DT_byte* slave_up) {
	if (*side == COM_CONF_LEFT) {
		*side = COM_CONF_RIGHT;

		leg_dwn = &leg_r;
		*slave_dwn = COM_CONF_LEFT;

		leg_up = &leg_l;
		*slave_up = COM_CONF_RIGHT;
	} else {
		*side = COM_CONF_LEFT;

		leg_dwn = &leg_l;
		*slave_dwn = COM_CONF_RIGHT;

		leg_up = &leg_r;
		*slave_up = COM_CONF_LEFT;
	}
}

DT_point ma_getPntForCpuSide(const DT_point* const p, const DT_byte cpuId,
		const DT_byte side) {
	DT_point pTmp = *p;
	if (cpuId == COM_SLAVE1B) {
		pTmp.y = p->y - MV_DST_Y;
	}
	if (cpuId == COM_SLAVE3F) {
		pTmp.y = p->y + MV_DST_Y;
	}
	return pTmp;
}

void master();
void ma_setInitialPoint(DT_point* const );
void ma_changeActiveLeg();
void ma_doStep(const DT_point* const );

int main() {
	XM_init_cpu();
	XM_init_dnx();
	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	XM_init_com(cpuID);

	XM_init_remote();

	XM_LED_OFF

	KIN_setTransMat(&leg_r);
	KIN_setTransMat(&leg_l);

	XM_LED_ON

	switch (cpuID) {
	case COM_MASTER:
		XM_init_remote();
		DEBUG(("Master",sizeof("Master")))
		master();
		break;
	case COM_SLAVE1B:
		DEBUG(("Slave1",sizeof("Slave1")))
		MV_slave(cpuID, &leg_r, &leg_l);
		break;
	case COM_SLAVE3F:
		DEBUG(("Slave3",sizeof("Slave3")))
		MV_slave(cpuID, &leg_r, &leg_l);
		break;
	default: //case NOCPUID:
		DEBUG (("NoCpuID",sizeof("NoCpuID")))
		XM_LED_OFF
		break;
	}

	return 0;
}

/* ___ Methoden fuer Master ___ */
void master() {
	DEBUG(("ma_chk_al",sizeof("ma_chk_al")))
	MV_masterCheckAlive();

	//DT_cmd cmd = 0x0000;

	MV_doInitPosition(&leg_r, &leg_l);

	DT_double zUp, zDwn;
	DT_point vDwn, vUp, pDwn, pUp, pTmp;
	vDwn.x = 0;
	vDwn.y = 0;
	vDwn.z = 0;

	vUp = ma_negateVec(&vDwn);
	DT_bool resDwn = true, resUp = true;
	DT_byte side = COM_CONF_LEFT;
	DT_byte config, slaveDwn, slaveUp;
	DT_leg leg_up, leg_dwn;

	while (1) {
		XM_LED_ON

		vUp = ma_calcUnitVec(&vUp);
		vDwn = ma_calcUnitVec(&vDwn);

		pDwn = ma_calcStartPoint(&vDwn, zDwn, slaveUp);
		pUp = ma_calcStartPoint(&vUp, zUp, slaveDwn);

		do {
			// TODO MV_point und COM_send auswerten ob Punkt angefahren werden kann
			config = slaveDwn;
			pTmp = ma_getPntForCpuSide(&pDwn, COM_SLAVE1B, slaveDwn);
			resDwn = resDwn && COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pDwn, COM_SLAVE3F, slaveDwn);
			resDwn = resDwn && COM_sendPoint(COM_SLAVE3F, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pDwn, COM_MASTER, slaveUp);
			//resDwn = resDwn && MV_point(&leg_dwn, &pTmp, false);
			MV_point(&leg_dwn, &pTmp, false);

			config = slaveUp;
			pTmp = ma_getPntForCpuSide(&pUp, COM_SLAVE1B, slaveUp);
			resUp = resUp && COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pUp, COM_SLAVE3F, slaveUp);
			resUp = resUp && COM_sendPoint(COM_SLAVE3F, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pUp, COM_MASTER, slaveDwn);
			//resUp = resUp && MV_point(&leg_up, &pTmp, false);
			MV_point(&leg_up, &pTmp, false);

			pDwn = ma_movePnt(&pDwn, &vDwn);
			pUp = ma_movePnt(&pUp, &vUp);
		} while (resDwn && resUp);
		ma_switchLegs(&side, &leg_dwn, &leg_up, &slaveDwn, &slaveUp);
		// TODO wechsle beine physisch

		/*
		 cmd = RMT_getCommand();

		 // Vorwärts & Rückwärts
		 if (RMT_isUpPressed(cmd)) {
		 DEBUG(("UP",sizeof("UP")))
		 p1.y = p1.y + STEP_SIZE;
		 }
		 if (RMT_isDownPressed(cmd)) {
		 DEBUG(("DWN",sizeof("DWN")))
		 p1.y = p1.y - STEP_SIZE;
		 }
		 // Links & Rechts
		 if (RMT_isLeftPressed(cmd)) {
		 DEBUG(("LFT",sizeof("LFT")))
		 p1.x = p1.x - STEP_SIZE;
		 }
		 if (RMT_isRightPressed(cmd)) {
		 DEBUG(("RGT",sizeof("RGT")))
		 p1.x = p1.x + STEP_SIZE;
		 }
		 // Hoch & Runter
		 if (RMT_isButton1Pressed(cmd)) {
		 DEBUG(("B1",sizeof("B1")))
		 p1.z = p1.z + STEP_SIZE;
		 }
		 if (RMT_isButton3Pressed(cmd)) {
		 DEBUG(("B3",sizeof("B3")))
		 p1.z = p1.z - STEP_SIZE;
		 }

		 ma_doStep(&p1);
		 */
	}
}

void ma_doStep(const DT_point* const point) {
	DT_byte config;

	config = COM_CONF_LEFT | COM_CONF_RIGHT;
	COM_sendPoint(COM_SLAVE1B, point, config);
	COM_sendPoint(COM_SLAVE3F, point, config);

	MV_point(&leg_l, point, false);
	MV_point(&leg_r, point, false);

	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void ma_setInitialPoint(DT_point* const initPoint) {
	initPoint->x = 77.8553;
	initPoint->y = 77.8553;
	initPoint->z = -129.1041;
}

#endif /* TEST_ON */
