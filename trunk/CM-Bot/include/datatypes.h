/**
 * \file	datatypes.h
 *
 * \brief	Abstrahiert Datentypen.
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdlib.h>

#define DT_RX_BUFFER_SIZE 255	/**< Größe des Ring-Buffers. */
#define DT_RESULT_BUFFER_SIZE 255	/**< Größe des Buffers für ein empfangenes Paket. */

typedef int DT_int;
typedef double DT_double;
typedef uint8_t DT_byte;
typedef uint16_t DT_size;
typedef uint8_t DT_type;
typedef char DT_char;

typedef struct {
	DT_byte id;
	DT_double set_value;
	DT_double act_value;
} DT_servo;

typedef struct {
	DT_servo hip;
	DT_servo knee;
	DT_servo foot;
} DT_leg;

typedef struct {
	DT_double x, y, z;
} DT_point;

/**
 * \brief	Ring-Buffer zum Empfangen von Daten einer USART.
 */
typedef struct {
	DT_byte putIndex; /**< Index, ab dem neue Daten eingefügt werden. */
	DT_byte getIndex; /**< Index, ab dem Daten gelesen werden. */
	DT_byte lastByteLength; /**< Größe des zuletzt gesendeten Pakets. */
	DT_byte overflow_flag; /**< Zeigt an, ob neue Daten am Anfang geschrieben und alte Daten am Ende gelesen werden. */
	DT_byte buffer[DT_RX_BUFFER_SIZE]; /**< Feld für gespeicherte Daten. */
} DT_rxBuffer;

#endif /* DATATYPES_H_ */
