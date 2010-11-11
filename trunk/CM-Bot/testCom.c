/**
 * \file	testCom.c
 *
 * \brief	Testprogramm für Kommunikation der CPUs.
 */

#define TEST_ON
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/communication.h"

DT_leg leg_r, leg_l;
DT_byte Own_CpuID;

void master();
void slave();

int main() {
	XM_init_cpu();
	XM_init_dnx();
	XM_init_com();
	//XM_init_remote();

	XM_LED_OFF

	DNX_getConnectedIDs(&leg_r, &leg_l);
	Own_CpuID = COM_getCpuID(&leg_l);

	switch (Own_CpuID) {
	case COM_MASTER:
		DEBUG(("Master",sizeof("Master")))
		break;
	case COM_SLAVE1:
		DEBUG(("Slave1",sizeof("Slave1")))
		break;
	case COM_SLAVE3:
		DEBUG(("Slave3",sizeof("Slave3")))
		break;
	default: //case NOCPUID:
		DEBUG(("NoCpuID",sizeof("NoCpuID")))
		break;
	}

	if (Own_CpuID == COM_MASTER) {
		master();
	} else {
		slave();
	}

	return 0;
}

void master() {
	DT_bool flag1 = false, flag2 = false;
	do {
		if (!flag1)
			flag1 = COM_isAlive(COM_SLAVE1);
		if (!flag2)
			flag2 = COM_isAlive(COM_SLAVE3);
		UTL_wait(4);
	} while (!flag1 || !flag2);

	XM_LED_ON

	DT_point p1, p2;

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	while (1) {
		XM_LED_OFF

		COM_sendPoint(COM_SLAVE1, &p1);
		COM_sendPoint(COM_SLAVE3, &p1);
		COM_sendAction(COM_BRDCAST_ID);

		UTL_wait(40);
		XM_LED_ON

		COM_sendPoint(COM_SLAVE1, &p2);
		COM_sendPoint(COM_SLAVE3, &p2);
		COM_sendAction(COM_BRDCAST_ID);

		UTL_wait(40);
	}
}

void slave() {
	DT_size len;
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_point p1;
	DT_bool ans;
	while (1) {
		len = COM_receive(&XM_com_data, result);
		if (len == 0)
			continue;
		DEBUG(("sl_pck_rec",sizeof("sl_pck_rec")))

		if (result[2] != Own_CpuID && result[2] != COM_BRDCAST_ID)
			continue;
		DEBUG(("sl_for_me",sizeof("sl_for_me")))

		switch (result[4]) {
		case COM_STATUS:
			switch (result[5]) {
			case COM_IS_ALIVE:
				XM_LED_ON
				COM_sendACK(COM_MASTER);
				DEBUG(("sl_alive",sizeof("sl_alive")))
				break;
			default:
				break;
			}
			break;
		case COM_ACTION:
			ans = KIN_makeMovement(&leg_l, &leg_r);
			if (ans)
				COM_sendACK(COM_MASTER);
			else
				COM_sendNAK(COM_MASTER, COM_ERR_DEFAULT_ERROR);
			break;
		case COM_POINT:
			// point aus Paket lesen
			p1 = COM_getPointFromPaket(result);

			KIN_calcServos(&p1, &leg_l);
			KIN_calcServos(&p1, &leg_r);

			COM_sendACK(COM_MASTER);
			break;

		default:
			// ERROR
			break;
		}
	}

}

#endif /* TEST_ON */
