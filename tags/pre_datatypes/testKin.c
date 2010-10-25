/**
 * \file	testKin.c
 *
 * \brief	Testprogramm für die Kinematik.
 */

#define TEST_ON
#ifdef TEST_ON

// #include <stdio.h>
// #include <stdlib.h>

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"

int main() {

#ifdef _x86
	double v1 = 0, v2 = 0, v3 = 0;
	int i;
	servos s1, s;
	point p1; p2;
	printf("< Denavid-Hardenberg - Roboterkoordinaten zu Weltkoordinaten>\n");
	while (1) {
		printf("> Hueftgelenk (in Grad): ");
		scanf("%lf", &v1);
		v1 = UTL_getRadiant(v1);
		printf("> Kniegelenk (in Grad): ");
		scanf("%lf", &v2);
		v2 = UTL_getRadiant(v2);
		printf("> Fuszgelenk (in Grad): ");
		scanf("%lf", &v3);
		v3 = UTL_getRadiant(v3);

		s1.v1 = v1;
		s1.v2 = v2;
		s1.v3 = v3;

		printf("--- Denavid-Hardenberg - Eingabe ---\n");
		UTL_printServos(s1, UTL_DEG);

		double** dh03 = malloc(KIN_ROWS * sizeof(double*));
		for (i = 0; i < KIN_ROWS; i++)
		dh03[i] = malloc(KIN_COLUMNS * sizeof(double));

		KIN_calculateDH(s1, dh03);
		UTL_printMatrix(dh03, KIN_ROWS, KIN_COLUMNS);
		p1 = UTL_getPointOfDH(dh03);
		UTL_printPoint(p1);

		printf("--- Inverses kin. Problem - Berechnung und Vergleich---\n");
		s = KIN_calculateServos(p1);
		UTL_printServos(s, UTL_DEG);
		UTL_printServos(s1, UTL_DEG);

		printf("--- Denavid-Hardenberg - Berechnung und Vergleich ---\n");
		KIN_calculateDH(s, dh03);
		p2 = UTL_getPointOfDH(dh03);

		UTL_printPoint(p2);
		UTL_printPoint(p1);

		for (i = 0; i < KIN_ROWS; i++)
		free(dh03[i]);
		free(dh03);
		printf("------------------------------------------------------\n");
	}
#endif
	XM_init_cpu();
	XM_init_dnx();

	XM_LED_OFF

	byte id;

	// double dh03[KIN_ROWS][KIN_COLUMNS];
	servos s;
	point p1, p2;

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	char flag = 0;
	while (1) {
		UTL_wait(40);
		if (flag == 0) {
			s = KIN_calculateServos(p1);
			flag = 1;
		} else {
			s = KIN_calculateServos(p2);
			flag = 0;
		}
		s.v1 = UTL_getDegree(s.v1);
		s.v2 = UTL_getDegree(s.v2);
		s.v3 = UTL_getDegree(s.v3);

		id = 10;
		DNX_setAngle(id, s.v1);
		id = 11;
		DNX_setAngle(id, s.v2);
		id = 12;
		DNX_setAngle(id, s.v3);
		id = 7;
		DNX_setAngle(id, s.v1);
		id = 8;
		DNX_setAngle(id, s.v2);
		id = 9;
		DNX_setAngle(id, s.v3);
		/*	if (!s.v1 || !s.v2 || !s.v3) {
		 DEBUG(("s falsch", sizeof("s falsch")))
		 return 0;
		 }
		 */
	}
	XM_LED_ON

	return 0;
}

#endif /* TEST_ON */
