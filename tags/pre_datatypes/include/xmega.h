/**
 * \file 	xmega.h
 * \brief 	Spezifische Funktionen für den Mikrocontroller ATXmega128A1.
 */

#ifndef XMEGA_H_
#define XMEGA_H_

#include "avr/io.h"
#include "avr_compiler.h"
#include "usart_driver.h"
#include "clksys_driver.h"
#include "utils.h"

#define XM_PORT_SERVO_L PORTC
#define XM_PORT_SERVO_R PORTD
#define XM_PORT_DEBUG PORTF

#define XM_USART_SERVO_L USARTC0
#define XM_USART_SERVO_R USARTD0
#define XM_USART_DEBUG USARTF0

/* LED */
#define XM_PORT_LED PORTQ
#define XM_LED_MASK (1<<PIN3)
#define XM_LED_ON XM_PORT_LED.OUTCLR = XM_LED_MASK;
#define XM_LED_OFF XM_PORT_LED.OUTSET = XM_LED_MASK;

/* Taster */
#define SWITCHPORT PORTQ     // Port PORTQ
#define SWITCHMASK (1<<PIN2) // Taster an PQ2
#define SWITCH_PRESSED (SWITCHPORT.IN&SWITCHMASK)!=SWITCHMASK
#define SWITCH_RELEASED (SWITCHPORT.IN&SWITCHMASK)==SWITCHMASK

#define XM_OE_MASK (1<<PIN0)

#define XM_RX_BUFFER_SIZE 255	/**< Größe des Ring-Buffers. */
#define XM_RESULT_BUFFER_SIZE 255	/**< Größe des Buffers für ein empfangenes Paket. */

USART_data_t XM_servo_data_L;	/**< USART-Struktur für linke Dynamixel. */
USART_data_t XM_servo_data_R;	/**< USART-Struktur für rechte Dynamixel. */
USART_data_t XM_debug_data;	/**< USART-Struktur für Debug-Ausgaben. */

/**
 * \brief	Ring-Buffer zum Empfangen von Daten einer USART.
 */
typedef struct {
	byte putIndex;	/**< Index, ab dem neue Daten eingefügt werden. */
	byte getIndex;	/**< Index, ab dem Daten gelesen werden. */
	byte lastByteLength;	/**< Größe des zuletzt gesendeten Pakets. */
	byte overflow_flag;	/**< Zeigt an, ob neue Daten am Anfang geschrieben und alte Daten am Ende gelesen werden. */
	byte buffer[XM_RX_BUFFER_SIZE]; /**< Feld für gespeicherte Daten. */
} RXBuffer;

RXBuffer XM_RX_buffer_L;	/**< Ring-Buffer für linke USART. */
RXBuffer XM_RX_buffer_R;	/**< Ring-Buffer für rechte USART. */

void XM_init_cpu();
void XM_init_dnx();
void XM_init_com();
void XM_USART_send(USART_data_t*, byte*, byte);
byte XM_USART_receive(RXBuffer*, byte*);

#endif /* XMEGA_H_ */
