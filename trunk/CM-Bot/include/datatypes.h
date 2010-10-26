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
	DT_byte id;				/**< Servo-ID. */
	DT_double set_value;    /**< Soll-Wert. */
	DT_double act_value;	/**< Ist-Wert. */
} DT_servo;					/**< Datenstruktur zur Speicherung von ID, Soll- und Ist-Wert eines Servos. */

typedef struct {
	DT_servo hip;			/**< Hüftgelenk. */
	DT_servo knee;			/**< Kniegelenk. */
	DT_servo foot;			/**< Fußgelenk. */
} DT_leg;					/**< Datenstruktur zur Speicherung von Servodaten bezüglich eines kompletten Beines. */

typedef struct {
	DT_double x, y, z;
} DT_point;                /**< Datenstruktur zur Speicherung karthesischer Koordinaten. */

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
