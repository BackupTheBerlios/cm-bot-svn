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

void master();
void ma_setInitialPoint(DT_point* const );
void ma_changeActiveLeg();
void ma_doStep();

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

	DEBUG(("ma_int_pnt",sizeof("ma_int_pnt")))
	DT_point p1;
	ma_setInitialPoint(&p1);
	DT_cmd cmd = 0x0000;

	MV_doInitPosition(&leg_r, &leg_l);

	while (1) {
		XM_LED_ON
		cmd = RMT_getCommand();
		if (RMT_isButton1Pressed(cmd)) {
			DEBUG(("B1",sizeof("B1")))
			p1.x = p1.x + 1;
		}

		if (RMT_isButton2Pressed(cmd)) {
			DEBUG(("B2",sizeof("B2")))
			p1.x = p1.x - 1;
		}

		if (RMT_isButton3Pressed(cmd)) {
			DEBUG(("B3",sizeof("B3")))
			p1.y = p1.y + 1;
		}

		if (RMT_isButton4Pressed(cmd)) {
			DEBUG(("B4",sizeof("B4")))
			p1.y = p1.y - 1;
		}
		if (RMT_isLeftPressed(cmd)) {
			DEBUG(("B1",sizeof("BL")))
			p1.y = p1.z + 1;
		}
		if (RMT_isRightPressed(cmd)) {
			DEBUG(("B1",sizeof("BR")))
			p1.y = p1.z - 1;
		}
		ma_doStep(p1);
	}
}

void ma_doStep(DT_point* point) {
	DT_byte config;
	DT_point pTmp;

	config = COM_CONF_LEFT | COM_CONF_RIGHT;
	COM_sendPoint(COM_SLAVE1B, point, config);
	COM_sendPoint(COM_SLAVE3F, point, config);

	MV_point(&leg_l, point, false);
	MV_point(&leg_r, point, false);

	//COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void ma_setInitialPoint(DT_point* const initPoint) {
	initPoint->x = 77.8553;
	initPoint->y = 77.8553;
	initPoint->z = -129.1041;
}

#endif /* TEST_ON */
