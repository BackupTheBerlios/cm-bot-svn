/*
 * testKin.c
 *
 * Created on: 24.09.2010
 * Author: christof
 */

#define TEST_OFF
#ifdef TEST_ON

#include <stdio.h>
#include <stdlib.h>

#include "include/kinematics.h"
#include "include/utils.h"

int main() {
	double v1 = 0, v2 = 0, v3 = 0;
	int i;
	servos s1, s2;
	point p1, p2;
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
		s2 = KIN_calculateServos(p1);
		UTL_printServos(s2, UTL_DEG);
		UTL_printServos(s1, UTL_DEG);

		printf("--- Denavid-Hardenberg - Berechnung und Vergleich ---\n");
		KIN_calculateDH(s2, dh03);
		p2 = UTL_getPointOfDH(dh03);

		UTL_printPoint(p2);
		UTL_printPoint(p1);

		for (i = 0; i < KIN_ROWS; i++)
			free(dh03[i]);
		free(dh03);
		printf("------------------------------------------------------\n");
	}

	return 0;
}

#endif /* TEST_ON */
