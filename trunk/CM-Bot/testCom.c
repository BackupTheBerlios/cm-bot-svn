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
	DT_bool flag;
	flag = COM_isAlive(COM_SLAVE1);
	DEBUG(("ma_alive_sent",sizeof("ma_alive_sent")))
	//COM_isAlive(COM_SLAVE2);

	if (flag)
		XM_LED_ON

	while (1) {

	}
}

void slave() {
	DT_size len;
	DT_byte result[DT_RESULT_BUFFER_SIZE];
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
				DEBUG(("sl_alive",sizeof("sl_alive")))
				XM_LED_ON
				COM_sendACK(COM_MASTER);
				break;
			default:
				break;
			}
			break;
		case COM_ACTION:

			break;
		case COM_POINT:

			break;

		default:
			// ERROR
			break;
		}
	}

}

#endif /* TEST_ON */
