/**
 * \file	testKin.c
 *
 * \brief	Testprogramm für die Kinematik.
 */

#define TEST_ON
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/remote.h"

int main() {
	XM_init_cpu();
	XM_init_remote();
	XM_init_dnx();

	XM_LED_OFF

	DT_byte receivedData[DT_RESULT_BUFFER_SIZE];
	DT_cmd cmd;

	DT_byte id;
	DT_leg leg;
	DT_point p1, p2;

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	DT_char flag = 0;
	while (1) {
		if (flag == 0) {
			leg = KIN_calculateServos(p1);
			flag = 1;
		} else {
			leg = KIN_calculateServos(p2);
			flag = 0;
		}
		leg.hip.set_value = UTL_getDegree(leg.hip.set_value);
		leg.knee.set_value = UTL_getDegree(leg.knee.set_value);
		leg.foot.set_value = UTL_getDegree(leg.foot.set_value);

		//cmd = getInstruction();
		UTL_wait(20);
		//if (cmd == B_1) {
		id = 10;
		DNX_setAngle(id, leg.hip.set_value);
		id = 11;
		DNX_setAngle(id, leg.knee.set_value);
		id = 12;
		DNX_setAngle(id, leg.foot.set_value);
		//}

		UTL_wait(20);

		//if (cmd == B_2) {
		id = 7;
		DNX_setAngle(id, leg.hip.set_value);
		id = 8;
		DNX_setAngle(id, leg.knee.set_value);
		id = 9;
		DNX_setAngle(id, leg.foot.set_value);

		//}

	}
	return 0;
}

#endif /* TEST_ON */
