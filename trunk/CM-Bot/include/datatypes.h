/**
 * \file	datatypes.h
 *
 * \brief	Abstrahiert Datentypen.
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define DT_RESULT_BUFFER_SIZE 128	/**< Größe des Buffers für ein empfangenes Paket. */

typedef bool DT_bool;
typedef int DT_int;
typedef double DT_double;
typedef uint8_t DT_byte;
typedef uint16_t DT_size;
typedef uint8_t DT_type;
typedef char DT_char;
typedef uint16_t DT_cmd;

/** \brief Datenstruktur zur Speicherung von ID, Soll- und Ist-Wert eines Servos. */
typedef struct {
	DT_byte id; /**< Servo-ID. */
	DT_double set_value; /**< Soll-Wert. */
	DT_double act_value; /**< Ist-Wert. */
} DT_servo;

/** \brief Struktur zur vereinfachten Koordinatentransformation. */
typedef struct {
	DT_double x, y;
	DT_bool zRotation;
} DT_transformation;

/** \brief Datenstruktur zur Speicherung von Servodaten bezüglich eines kompletten Beines. */
typedef struct {
	DT_servo hip; /**< Hüftgelenk. */
	DT_servo knee; /**< Kniegelenk. */
	DT_servo foot; /**< Fußgelenk. */
	DT_transformation trans; /**< Infos fuer Koordinatentransformation. */
} DT_leg;

/** \brief Datenstruktur zur Speicherung karthesischer Koordinaten. */
typedef struct {
	DT_double x, y, z;
} DT_point;

#endif /* DATATYPES_H_ */
