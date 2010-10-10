/*
 * kinematics.c
 *
 * Created on: 27.09.2010
 * Author: christof
 */

#include <math.h>
#include <stdlib.h>

#include "include/kinematics.h"
#include "include/utils.h"

#define DIST_HK 50
#define DIST_KF 85
#define DIST_FE 55
#define DIST_DZ -14

void KIN_calculateDH(const servos s, double** dh03) {
	dh03[0][0] = cos(s.v1) * cos(s.v2) * cos(s.v3) - cos(s.v1) * sin(s.v2)
			* sin(s.v3);
	dh03[0][1] = -cos(s.v1) * cos(s.v2) * sin(s.v3) - cos(s.v1) * cos(s.v3)
			* sin(s.v2);
	dh03[0][2] = -sin(s.v1);
	dh03[0][3] = 50 * cos(s.v1) + 85 * cos(s.v1) * cos(s.v2) - 55 * cos(s.v1)
			* sin(s.v2) * sin(s.v3) + 55 * cos(s.v1) * cos(s.v2) * cos(s.v3);

	dh03[1][0] = cos(s.v2) * cos(s.v3) * sin(s.v1) - sin(s.v1) * sin(s.v2)
			* sin(s.v3);
	dh03[1][1] = -cos(s.v2) * sin(s.v1) * sin(s.v3) - cos(s.v3) * sin(s.v1)
			* sin(s.v2);
	dh03[1][2] = cos(s.v1);
	dh03[1][3] = 50 * sin(s.v1) + 85 * cos(s.v2) * sin(s.v1) - 55 * sin(s.v1)
			* sin(s.v2) * sin(s.v3) + 55 * cos(s.v2) * cos(s.v3) * sin(s.v1);

	dh03[2][0] = -cos(s.v2) * sin(s.v3) - cos(s.v3) * sin(s.v2);
	dh03[2][1] = sin(s.v2) * sin(s.v3) - cos(s.v2) * cos(s.v3);
	dh03[2][2] = 0;
	dh03[2][3] = -85 * sin(s.v2) - 55 * cos(s.v2) * sin(s.v3) - 55 * cos(s.v3)
			* sin(s.v2) - 14;

	dh03[3][0] = 0;
	dh03[3][1] = 0;
	dh03[3][2] = 0;
	dh03[3][3] = 1;
}

servos KIN_calculateServos(const point p) {
	servos servos;
	double z = p.z - DIST_DZ;
	double h, h2, h3;
	double v1, v2, v3;
	double alpha, beta, gamma;

	// STEP 1 (without dummy-axis)
	// angle for hip axis in x-y-plane
	h = sqrt(p.x * p.x + p.y * p.y);
	// v1 = asin(p.y / h);
	v1 = atan(p.y / p.x); // should have better precision

	// STEP 2
	// angle for hip & foot axis in z-h' plane
	h2 = h - DIST_HK;
	h3 = sqrt(h2 * h2 + z * z);

	/* FIXME DEBUG(printf...
	DEBUG(printf("calculateServos(): v1 = %lf\n", v1);)
	DEBUG(printf("calculateServos(): h = %lf\n", h);)
	DEBUG(printf("calculateServos(): h2 = %lf\n", h2);)
	DEBUG(printf("calculateServos(): h3 = %lf\n", h3);)
	*/

	alpha = h2 != h3 ? acos(
			(-(h3 * h3) + DIST_FE * DIST_FE + DIST_KF * DIST_KF) / (2 * DIST_FE
					* DIST_KF)) : M_PI; // law of cosine
	beta = asin((DIST_FE / h3) * sin(alpha)); // law of sines
	gamma = asin(abs(z) / h3); // rules of right angle triangle, abs(z) 'cause length of trianglearm!

	/* FIXME DEBUG(printf...
	DEBUG(printf("calculateServos(): alpha = %lf\n", alpha);)
	DEBUG(printf("calculateServos(): beta = %lf\n", beta);)
	DEBUG(printf("calculateServos(): gamma = %lf\n", gamma);)
	*/

	// CASES
	if (z < 0) { // defined for z < 0: foot-axis is between h3 and h-axis
		v2 = gamma - beta;
	} else { // defined for z >= 0: foot-axis is not between h3 and h-axis
		v2 = -gamma - beta;
	}
	v3 = M_PI - alpha;

	/*
	DEBUG(printf("calculateServos(): v2 = %lf\n", v2);)
	DEBUG(printf("calculateServos(): v3 = %lf\n", v3);)
	*/

	servos.v1 = v1;
	servos.v2 = v2;
	servos.v3 = v3;

	return servos;
}
