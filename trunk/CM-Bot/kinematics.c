/**
 * \file	kinematics.c
 *
 * \brief	Lösungsmethoden der Kinematik.
 *
 * 			Lösungsmethoden der Kinematik speziell für den CM-Bot.
 */

#include <math.h>
#include <stdlib.h>

#include "include/kinematics.h"
#include "include/utils.h"

/**
 * \def	DIST_HK
 * \brief	Abstand von Hüfte zu Knie.
 *
 * \def	DIST_KF
 * \brief	Abstand von Knie zu Fuß.
 *
 * \def	DIST_FE
 * \brief	Abstand von Fuß zu Fußende.
 *
 * \def	DIST_DZ
 * \brief	Versatz in z-Richtung von Knie in Bezug auf Hüfte.
 */
#define DIST_HK 50
#define DIST_KF 85
#define DIST_FE 55
#define DIST_DZ -14

/**
 * \brief	Lösung des kinematischen Problems.
 *
 * 			Lösung der Denavit-Hartenberg-Transformation.
 *
 * \param	leg		Bein mit den Soll-Winkel der Gelenke
 * \param	dh03	Zielmatrix für die Lösung
 */
void KIN_calculateDH(const DT_leg leg, DT_double** dh03) {
	dh03[0][0] = cos(leg.hip.set_value) * cos(leg.knee.set_value) * cos(leg.foot.set_value)
			- cos(leg.hip.set_value) * sin(leg.knee.set_value) * sin(leg.foot.set_value);
	dh03[0][1] = -cos(leg.hip.set_value) * cos(leg.knee.set_value) * sin(leg.foot.set_value)
			- cos(leg.hip.set_value) * cos(leg.foot.set_value) * sin(leg.knee.set_value);
	dh03[0][2] = -sin(leg.hip.set_value);
	dh03[0][3] = 50 * cos(leg.hip.set_value) + 85 * cos(leg.hip.set_value) * cos(
			leg.knee.set_value) - 55 * cos(leg.hip.set_value) * sin(leg.knee.set_value) * sin(
			leg.foot.set_value) + 55 * cos(leg.hip.set_value) * cos(leg.knee.set_value) * cos(
			leg.foot.set_value);

	dh03[1][0] = cos(leg.knee.set_value) * cos(leg.foot.set_value) * sin(leg.hip.set_value)
			- sin(leg.hip.set_value) * sin(leg.knee.set_value) * sin(leg.foot.set_value);
	dh03[1][1] = -cos(leg.knee.set_value) * sin(leg.hip.set_value) * sin(leg.foot.set_value)
			- cos(leg.foot.set_value) * sin(leg.hip.set_value) * sin(leg.knee.set_value);
	dh03[1][2] = cos(leg.hip.set_value);
	dh03[1][3] = 50 * sin(leg.hip.set_value) + 85 * cos(leg.knee.set_value) * sin(
			leg.hip.set_value) - 55 * sin(leg.hip.set_value) * sin(leg.knee.set_value) * sin(
			leg.foot.set_value) + 55 * cos(leg.knee.set_value) * cos(leg.foot.set_value) * sin(
			leg.hip.set_value);

	dh03[2][0] = -cos(leg.knee.set_value) * sin(leg.foot.set_value) - cos(leg.foot.set_value)
			* sin(leg.knee.set_value);
	dh03[2][1] = sin(leg.knee.set_value) * sin(leg.foot.set_value) - cos(leg.knee.set_value)
			* cos(leg.foot.set_value);
	dh03[2][2] = 0;
	dh03[2][3] = -85 * sin(leg.knee.set_value) - 55 * cos(leg.knee.set_value) * sin(
			leg.foot.set_value) - 55 * cos(leg.foot.set_value) * sin(leg.knee.set_value) - 14;

	dh03[3][0] = 0;
	dh03[3][1] = 0;
	dh03[3][2] = 0;
	dh03[3][3] = 1;
}

/**
 * \brief	Lösung des inversen kinematischen Problems.
 *
 * 			Lösung des inversen kinematischen Problems mit Hilfe eines geometrischen Verfahrens mit leichten Einschränkungen.
 *
 * \param	p	Punkt (Roboterkoorinate)
 *
 * \return	Bein mit den errechneten Soll-Winkel für hip, knee und foot
 */
DT_leg KIN_calculateServos(const DT_point p) {
	DT_leg leg;
	DT_double z = p.z - DIST_DZ;
	DT_double h, h2, h3;
	DT_double hip, knee, foot;
	DT_double alpha, beta, gamma;

	// STEP 1 (without dummy-axis)
	// angle for hip axis in x-y-plane
	h = sqrt(p.x * p.x + p.y * p.y);
	// v1 = asin(p.y / h);
	hip = atan(p.y / p.x); // should have better precision

	// STEP 2
	// angle for hip & foot axis in z-h' plane
	h2 = h - DIST_HK;
	h3 = sqrt(h2 * h2 + z * z);

	alpha = h2 != h3 ? acos(
			(-(h3 * h3) + DIST_FE * DIST_FE + DIST_KF * DIST_KF) / (2 * DIST_FE
					* DIST_KF)) : M_PI; // law of cosine
	beta = asin((DIST_FE / h3) * sin(alpha)); // law of sines
	gamma = asin(abs(z) / h3); // rules of right angle triangle, abs(z) 'cause length of trianglearm!

	// CASES
	if (z < 0) { // defined for z < 0: foot-axis is between h3 and h-axis
		knee = gamma - beta;
	} else { // defined for z >= 0: foot-axis is not between h3 and h-axis
		knee = -gamma - beta;
	}
	foot = M_PI - alpha;

	leg.hip.set_value = hip;
	leg.knee.set_value = knee;
	leg.foot.set_value = foot;

	return leg;
}
