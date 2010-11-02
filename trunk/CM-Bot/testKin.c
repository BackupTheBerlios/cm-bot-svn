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

int main() {
	XM_init_cpu();
	XM_init_dnx();

	XM_LED_ON

	DT_leg leg_r, leg_l;
	DT_point p1, p2;

	leg_r.hip.id = 0x07;
	leg_r.knee.id = 0x08;
	leg_r.foot.id = 0x09;

	leg_l.hip.id = 0x0A;
	leg_l.knee.id = 0x0B;
	leg_l.foot.id = 0x0C;

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	DT_char flag = 0;
	while (1) {
		UTL_wait(50);
		if (flag == 0) {
			KIN_calculateServos(&p1, &leg_l);
			KIN_calculateServos(&p1, &leg_r);
			flag = 1;
			XM_LED_ON
		} else {
			KIN_calculateServos(&p2, &leg_l);
			KIN_calculateServos(&p2, &leg_r);
			flag = 0;
			XM_LED_OFF
		}

		leg_r.hip.set_value = UTL_getDegree(leg_r.hip.set_value);
		leg_r.knee.set_value = UTL_getDegree(leg_r.knee.set_value);
		leg_r.foot.set_value = UTL_getDegree(leg_r.foot.set_value);

		leg_l.hip.set_value = UTL_getDegree(leg_l.hip.set_value);
		leg_l.knee.set_value = UTL_getDegree(leg_l.knee.set_value);
		leg_l.foot.set_value = UTL_getDegree(leg_l.foot.set_value);

		DNX_setAngle(leg_r.hip.id, leg_r.hip.set_value);
		DNX_setAngle(leg_r.knee.id, leg_r.knee.set_value);
		DNX_setAngle(leg_r.foot.id, leg_r.foot.set_value);

		DNX_setAngle(leg_l.hip.id, leg_l.hip.set_value);
		DNX_setAngle(leg_l.knee.id, leg_l.knee.set_value);
		DNX_setAngle(leg_l.foot.id, leg_l.foot.set_value);
	}
	XM_LED_OFF

	return 0;
}

#endif /* TEST_ON */
