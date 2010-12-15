/**
 *	\file evolutionaryAlgorithms.h
 *
 */


#include "datatypes.h"

DT_individuum evolutionaryAlgorithm(const DT_int popsize, const DT_int generations, DT_vector* const);
DT_point getPointFromIndividuum(DT_individuum *);
DT_point getIsectFromIndividuum(DT_individuum *);
