/**
 * \file 	xmega.h
 *
 * \brief 	Spezifische Funktionen für den Mikrocontroller ATXmega128A1.
 */

#ifndef XMEGA_H_
#define XMEGA_H_

#include "usart_driver.h"
#include "datatypes.h"

#define XM_PORT_SERVO_L PORTC
#define XM_PORT_SERVO_R PORTD
#define XM_PORT_REMOTE PORTE
#define XM_PORT_DEBUG PORTF

#define XM_USART_SERVO_L USARTC0
#define XM_USART_SERVO_R USARTD0
#define XM_USART_REMOTE USARTE1
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

USART_data_t XM_servo_data_L;	/**< USART-Struktur für linke Dynamixel. */
USART_data_t XM_servo_data_R;	/**< USART-Struktur für rechte Dynamixel. */
USART_data_t XM_debug_data;		/**< USART-Struktur für Debug-Ausgaben. */
USART_data_t XM_remote_data;	/**< USART-Struktur für Remote-Controller. */

DT_rxBuffer XM_RX_buffer_L;	/**< Ring-Buffer für linke USART. */
DT_rxBuffer XM_RX_buffer_R;	/**< Ring-Buffer für rechte USART. */

DT_rxBuffer XM_RX_remote; 	/**< Ring-Buffer für den Remote-Controller */

void XM_init_cpu();
void XM_init_remote();
void XM_init_dnx();
void XM_init_com();
void XM_USART_send(USART_data_t*, DT_byte*, DT_size);
DT_byte XM_USART_receive(DT_rxBuffer*, DT_byte*);

#endif /* XMEGA_H_ */
