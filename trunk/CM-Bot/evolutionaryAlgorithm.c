/**
 * \file	evolutionaryAlgorithm.c
 *
 * \brief	Evolutionärer Algorithmus zur Startpunktfindung.
 *
 * 			Startpunkfindung auf Basis eines Evolutionären Algorithmus.
 */

#include "include/datatypes.h"
#include "include/evolutionaryHelper.h"
#include "include/evolutionaryAlgorithm.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define X_MIN -83
#define X_MAX 83
#define Y_MIN 42
#define Y_MAX 129

// lokale Methoden
void generatePopulation(DT_individuum * const , const DT_int);
void printPopulation(DT_individuum*, DT_int);
DT_point generatePoint(DT_int);
DT_double getRandomNumber(DT_int, DT_int);
void fitnessproportionalSelection(const DT_individuum * const , DT_int*,
		const DT_int, const DT_int);
void recombination(const DT_individuum const *, DT_individuum * const ,
		const DT_int const *, const DT_int, const DT_int);
void uniformCrossover(const DT_individuum const *, const DT_individuum const *,
		DT_individuum *, DT_individuum *);
void mutation(DT_individuum *, const DT_int);
void gleichverteilte_reellwertige_mutation(DT_point *, const DT_double);
void bestSelection(DT_individuum * const , DT_individuum *, const DT_int,
		const DT_int);
void getScores(DT_vector* const , DT_individuum* const , const DT_int);


/**
 * \brief	Evolutionärer Algorithmus zur Startpunktfindung.
 *
 * 			Startpunktfindung auf Basis eines Evolutionären Algorithmus.
 *
 * \param	popsize		Größe der Population
 * \param	generations	Anzahl der Iterationen (Generationen)
 * \param	v			Vektor, der abgelaufen werden soll
 * \return	Bestes Individuum
 */
DT_individuum evolutionaryAlgorithm(const DT_int popsize,
		const DT_int generations, DT_vector* const v) {
	initEvoAlg();
	// Drehen des Koordinatensystems; - invertieren des Vektors, da der Weg zurückgelegt werden soll
	DT_vector v_inv;
	v_inv.y = - v->x;
	v_inv.x = - v->y;
	// Anzahl der Eltern pro Generation
	DT_int parentCnt = 10;
	// Population
	DT_individuum P[popsize];
	// Initialisierung
	generatePopulation(P, popsize);
	// Bewertung
	getScores(&v_inv, P, popsize);
	DT_int t = 0;
	while (generations > t) {
		// Index-Feld für Elternselektion
		DT_int I[parentCnt];
		// Fitnessproportionale Selektion
		fitnessproportionalSelection(P, I, popsize, parentCnt);
		DT_int popsizeNextGen = popsize + 2 * parentCnt;
		DT_individuum P_nextGen[popsizeNextGen];
		recombination(P, P_nextGen, I, popsize, parentCnt);
		mutation(P_nextGen, popsizeNextGen);
		getScores(&v_inv, P_nextGen, popsizeNextGen);
		t++;
		bestSelection(P_nextGen, P, popsizeNextGen, popsize);
		printPopulation(P, popsize);
	}
	return P[0];
}

DT_point getPointFromIndividuum(DT_individuum * A){
	DT_point p;
	p.x = A->G.y;
	p.y = A->G.x;
	p.z = A->G.z;
	return p;
}
/**
 * \brief	Population erzeugen.
 *
 * 			Erzeugt randomisierte Individuen, die innerhalb der Fläche liegen.
 *
 * \param	P		Pointer auf Feld, auf das die Individuen geschrieben werden sollen
 * \param	popsize	Größe der Population
 */
void generatePopulation(DT_individuum * const P, const DT_int popsize) {
	DT_int prob = 123;
	int i;
	for (i = 0; i < popsize; i++) {
		P[i].G = generatePoint(prob);
		prob += prob;
	}
}

/**
 * \brief	Ausgabe der Population.
 */
void printPopulation(DT_individuum* P, DT_int size) {
	int i;
	for (i = 0; i < size; i++)
		printf("P[%d]: x = %f\ty = %f\tA.F = %f\n", i, P[i].G.x, P[i].G.y,
				P[i].F);
}

/**
 * \brief	Punkt für Individuum erzeugen.
 *
 * 			Erzeugt randomisiert einen x- und einen y-Wert für den Punkt eines Individuums.
 *
 * \param	prob	Wert, um die Entropie zu erhöhen
 */
DT_point generatePoint(DT_int prob) {
	DT_point p;
	DT_size iseed = prob;
	do {
		iseed += prob;
		prob += prob;
		srand(iseed);
		p.x = getRandomNumber(X_MIN, X_MAX);
		iseed += prob;
		prob += prob;
		srand(iseed);
		p.y = getRandomNumber(Y_MIN, Y_MAX);
		p.z = Z;
	} while (isInArea(&p) != true);
	return p;
}

/**
 * \brief	Liefert einen Zufallswert.
 *
 *			Zufallswert zwischen den Schranken.
 *
 * \param	min		untere Schranke
 * \param	max		obere Schranke
 */
DT_double getRandomNumber(DT_int min, DT_int max) {
	double random;
	random = (rand() % (max - min + 1)) + min;
	return random;
}

/**
 * \brief	Fitnessproportionale Selektion.
 *
 *			Elternselektion auf Basis der Fitnessproportionalen Selektion.
 *
 * \param	P			Pointer auf Feld mit Individuen.
 * \param	I			Pointer auf Index-Feld, in dem die Indizes der Eltern-Individuen gespeichert werden soll.
 * \param	popsize		Größe der Population
 * \param	parentCnt	Anzahl der zu wählenden Eltern-Individuen
 */
void fitnessproportionalSelection(const DT_individuum * const P, DT_int* I,
		const DT_int popsize, const DT_int parentCnt) {
	DT_int i;
	DT_double sum[popsize], summe;
	sum[0] = 0;
	for (i = 1; i < popsize; i++) {
		sum[i] = sum[i - 1] + P[i - 1].F;
		summe = sum[i];
	}
	printf("\ndurchschnittliche Güte: %f\n", sum[popsize - 1] / popsize);
	for (i = 0; i < parentCnt; i++) {
		DT_int j = 1;
		DT_double u = getRandomNumber(0, sum[popsize - 1]);
		while (sum[j] < u) {
			j++;
		}
		I[i] = j;
	}
}

/**
 * \brief	Rekombination.
 *
 *			Rekombination der selektierten Eltern-Individuen - dabei werden bisherige Eltern zunächst übernommen.
 *
 * \param	P			Pointer auf Feld mit Individuen.
 * \param	P_nextGen	Pointer auf Feld mit Individuen der nächsten Generation
 * \param	I			Pointer auf Index-Feld, in dem die Indizes der Eltern-Individuen gespeichert werden soll.
 * \param	popsize		Größe der Population
 * \param	parentCnt	Anzahl der zu wählenden Eltern-Individuen
 */
void recombination(const DT_individuum const * P,
		DT_individuum * const P_nextGen, const DT_int const * I,
		const DT_int popsize, const DT_int parentCnt) {
	DT_int i;
	// Übernehmen der Eltern in neue Population
	for (i = 0; i < popsize; i++)
		P_nextGen[i] = P[i];
	// Rekombination
	for (i = 0; i < parentCnt; i++) {
		DT_int partner = I[i];
		uniformCrossover(&P[i], &P[partner], &P_nextGen[popsize + (i * 2)],
				&P_nextGen[popsize + (i * 2) + 1]);
	}
}

void uniformCrossover(const DT_individuum const * A,
		const DT_individuum const * B, DT_individuum * C, DT_individuum * D) {
	// l = 2, da 2-Dimensionales Problem
	do {
		DT_double b = getRandomNumber(0, 1);
		if (b == 0) {
			C->G.x = A->G.x;
			D->G.x = B->G.x;
		} else {
			C->G.x = B->G.x;
			D->G.x = A->G.x;
		}
		b = getRandomNumber(0, 1);
		if (b == 0) {
			C->G.y = A->G.y;
			D->G.y = B->G.y;
		} else {
			C->G.y = B->G.y;
			D->G.y = A->G.y;
		}
	} while ((isInArea(&C->G) != true) || (isInArea(&D->G) != true));
	C->G.z = Z;
	D->G.z = Z;
}

void mutation(DT_individuum * P, const DT_int popsize) {
	DT_int i;
	DT_double pm = 0.1;
	for (i = 0; i < popsize; i++) {
		gleichverteilte_reellwertige_mutation(&P[i].G, pm);
	}
}

void gleichverteilte_reellwertige_mutation(DT_point * A, const DT_double pm) {
	DT_double u = getRandomNumber(0, 10000) / 10000;
	DT_double unten, oben, smax = 30;
	if (u <= pm) {
		//printf("\nMutate:\nBefore: A.x = %f\tA.y = %f\n", A->x, A->y);
		do {
			unten = max(X_MIN + 20, A->x - smax);
			oben = min(X_MAX - 20, A->x + smax);
			A->x = getRandomNumber(unten, oben);
			unten = max(Y_MIN + 20, A->y - smax);
			oben = min(Y_MAX - 20, A->y + smax);
			A->y = getRandomNumber(unten, oben);
		} while (isInArea(A) != true);
		//printf("After: A.x = %f\tA.y = %f\n", A->x, A->y);
	}
}

void bestSelection(DT_individuum * const P_nextGen, DT_individuum * P,
		const DT_int popsizeNextGen, const DT_int popsize) {
	DT_int i;
	bubblesort(P_nextGen, popsizeNextGen);
	for (i = 0; i < popsize; i++) {
		if (P_nextGen[i].F > 1000)
			continue;
		P[i] = P_nextGen[i];
	}
}

void getScores(DT_vector* const v, DT_individuum* const P, const DT_int popsize) {
	DT_int i;
	for (i = 0; i < popsize; i++) {
		P[i].F = scorePoint(v, &P[i].G);
	}
}

// Todo:
// Elternselektion
// Rekombination
// Selektion
// Bewertungsfunktion
