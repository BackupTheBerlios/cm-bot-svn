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

int main() {
	XM_init_cpu();
	XM_init_dnx();

	XM_LED_ON

	DT_leg leg_r, leg_l;
	DT_byte CpuID;

	DNX_getConnectedIDs(&leg_r, &leg_l);
	CpuID = COM_getCpuID(&leg_r);

	while (1) {
		switch (CpuID) {
		case MASTER:
			//
			break;

		case SLAVE1:
			break;

		case SLAVE2:
			break;

		case NOCPUID:
			break;
		}
	}
	XM_LED_OFF

	return 0;
}

#endif /* TEST_ON */
