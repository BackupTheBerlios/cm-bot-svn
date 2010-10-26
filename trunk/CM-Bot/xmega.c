/**
 * \file 	xmega.c
 * \brief 	Spezifische Funktionen für den Mikrocontroller ATXmega128A1.
 */

#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/utils.h"
#include "include/clksys_driver.h"
#include "include/avr_compiler.h"
#include <avr/io.h>
#include <stdlib.h>

/**
 * \brief 	Initialisierung der CPU.
 */
void XM_init_cpu() {
	/******************************************************************
	 * System Clock 32MHz (XOSC Quarz 16MHz, PLL Faktor 2)
	 ******************************************************************/

	/* Nach dem Reset ist die Quelle des Systemtaktes der interne
	 2MHz RC-Oszillator (System Clock Selection: RC2MHz)
	 */

	// Oszillator XOSC konfigurieren (12..16MHz, 256 clocks startup time)
	CLKSYS_XOSC_Config(OSC_FRQRANGE_12TO16_gc, false,
			OSC_XOSCSEL_XTAL_256CLK_gc);

	// Oszillator XOSC enable
	CLKSYS_Enable(OSC_XOSCEN_bm);

	// Warten bis der Oszillator bereit ist
	do {
	} while (CLKSYS_IsReady(OSC_XOSCRDY_bm) == 0);

	// PLL source ist XOSC, Multiplikator x2
	CLKSYS_PLL_Config(OSC_PLLSRC_XOSC_gc, 2);

	// Enable PLL
	CLKSYS_Enable(OSC_PLLEN_bm);

	// Prescalers konfigurieren
	CLKSYS_Prescalers_Config(CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);

	// Warten bis PLL locked
	do {
	} while (CLKSYS_IsReady(OSC_PLLRDY_bm) == 0);

	// Main Clock Source ist Ausgang von PLL
	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_PLL_gc);

	// Nun ist der System Clock 32MHz !

	/* Hinweis:
	 32kHz TOSC kann nicht in Verbindung mit PLL genutzt werden, da
	 die minimale Eingangsfrequenz des PLLs 400kHz betraegt.
	 */

	/******************************************************************
	 * Debug-Usart initialisieren, 8N1 250kBit
	 ******************************************************************/
	XM_PORT_DEBUG.DIRSET = PIN3_bm; // Pin3 von PortF (TXD0) ist Ausgang
	XM_PORT_DEBUG.DIRCLR = PIN2_bm; // Pin2 von PortF (RXD0) ist Eingang

	// Use USART and initialize buffers
	USART_InterruptDriver_Initialize(&XM_debug_data, &XM_USART_DEBUG,
			USART_DREINTLVL_OFF_gc);
	// USARTF0, 8 Data bits, No Parity, 1 Stop bit.
	USART_Format_Set(XM_debug_data.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);

	/* Bitrate einstellen

	 Beispiele BSEL in Abhaengigkeit von der Bitrate, fck = 32MHz, Error < 0,8%

	 7 = 250.000bps
	 30 = 128.000bps
	 34 =  57.600bps
	 51 =  38.400bps
	 68 =  28.800bps
	 103 =  19.200bps
	 138 =  14.400bps
	 207 =   9.600bps
	 416 =   4.800bps
	 832 =   2.400bps
	 1666 =   1.200bps

	 Bemerkung: Geprueft wurde mit 250.000bps im USBxpress Modus
	 */

	USART_Baudrate_Set(XM_debug_data.usart, 7, 0); // 250.000bps (BSEL = 7)

	/* Enable RX and TX. */
	USART_Rx_Enable(XM_debug_data.usart);
	USART_Tx_Enable(XM_debug_data.usart);

	USART_GetChar(XM_debug_data.usart); // Flush Receive Buffer
	DEBUG(("DEBUG-USART ... ON", sizeof("DEBUG-USART ... ON")));

	// Init LED
	XM_PORT_LED.DIRSET = XM_LED_MASK;
	XM_LED_ON

	// Init Taster
	SWITCHPORT.DIRCLR = SWITCHMASK;
	SWITCHPORT.PIN2CTRL |= (0b011 << 3); // Pullup PQ2 aktivieren
}

/**
 * \brief 	Initialisiert die Servo-USARTs.
 */
void XM_init_dnx() {
	//Disable Interrupts
	cli();

	// Init buffer
	XM_RX_buffer_L.getIndex = 0;
	XM_RX_buffer_L.putIndex = 0;
	XM_RX_buffer_L.overflow_flag = 0x00;

	XM_RX_buffer_R.getIndex = 0;
	XM_RX_buffer_R.putIndex = 0;
	XM_RX_buffer_R.overflow_flag = 0x00;

	// Set pins for TX and RX
	XM_PORT_SERVO_R.DIRSET = PIN3_bm; // Pin3 of PortC (TXD0) is output
	XM_PORT_SERVO_R.DIRCLR = PIN2_bm; // Pin2 of PortC (RXD0) is input

	XM_PORT_SERVO_L.DIRSET = PIN3_bm;
	XM_PORT_SERVO_L.DIRCLR = PIN2_bm;

	// Set pin, dir and out for OE
	XM_PORT_SERVO_R.DIRSET = XM_OE_MASK;
	XM_PORT_SERVO_R.OUTSET = XM_OE_MASK;

	XM_PORT_SERVO_L.DIRSET = XM_OE_MASK;
	XM_PORT_SERVO_L.OUTSET = XM_OE_MASK;

	// Use USARTC0 / USARTD0 and initialize buffers
	USART_InterruptDriver_Initialize(&XM_servo_data_R, &XM_USART_SERVO_R,
			USART_DREINTLVL_OFF_gc);
	USART_InterruptDriver_Initialize(&XM_servo_data_L, &XM_USART_SERVO_L,
			USART_DREINTLVL_OFF_gc);

	// 8 Data bits, No Parity, 1 Stop bit
	USART_Format_Set(XM_servo_data_R.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);
	USART_Format_Set(XM_servo_data_L.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);

	// Enable DRE interrupt
	// USART_DreInterruptLevel_Set(XM_servo_data_R.usart, USART_DREINTLVL_LO_gc);
	// USART_DreInterruptLevel_Set(XM_servo_data_L.usart, USART_DREINTLVL_LO_gc);

	// Enable TXC interrupt
	// USART_TxdInterruptLevel_Set(XM_servo_data_R.usart, USART_TXCINTLVL_LO_gc);
	// USART_TxdInterruptLevel_Set(XM_servo_data_L.usart, USART_TXCINTLVL_LO_gc);

	// Enable RXC interrupt
	USART_RxdInterruptLevel_Set(XM_servo_data_R.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(XM_servo_data_L.usart, USART_RXCINTLVL_LO_gc);

	// Set Baudrate
	USART_Baudrate_Set(XM_servo_data_R.usart, 1, 0); // 1 Mbps (BSEL = 1)
	USART_Baudrate_Set(XM_servo_data_L.usart, 1, 0); // 1 Mbps (BSEL = 1)

	// Enable RX and TX
	USART_Rx_Enable(XM_servo_data_R.usart);
	USART_Rx_Enable(XM_servo_data_L.usart);

	USART_Tx_Enable(XM_servo_data_R.usart);
	USART_Tx_Enable(XM_servo_data_L.usart);

	// Flush Receive Buffer
	USART_GetChar(XM_servo_data_R.usart); // Flush Receive Buffer
	USART_GetChar(XM_servo_data_L.usart); // Flush Receive Buffer

	// Enable PMIC interrupt level low
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	// Enable global interrupts
	sei();

}

/**
 * \brief 	Initialisiert USARTs für die CPU-Kommunikation.
 */
void XM_init_com() {
	// TODO
}

/**
 * \brief 	USART-Sendemethode.
 *
 * 			Diese Methode setzt zunächst das jeweilige Output-Enable (!OE) auf Senden
 * 			und schreibt das zu sendende Paket in den USART-Buffer.
 * 			Anschließend wird der TX-Interrupt aktiviert, der ausgelöst wird, wenn das
 * 			letzte Paket gesendet wurde.
 *
 * \param	usart_data	USART-Datenstruktur der zu benutzenden USART
 * \param	txData		Byte-Array mit zu sendendem Paket
 * \param	bytes 		Länge des zu sendenden Pakets
 */
void XM_USART_send(USART_data_t* usart_data, DT_byte* txData, DT_size bytes) {
	DT_size i;

	DEBUG_BYTE((txData, bytes))

	if (usart_data->usart == &XM_USART_DEBUG)
		return;

	// Set OE to 0 -> Enable Send
	if (usart_data->usart == &XM_USART_SERVO_L) {
		XM_RX_buffer_L.lastByteLength = bytes;
		XM_PORT_SERVO_L.OUTCLR = XM_OE_MASK;
	}
	if (usart_data->usart == &XM_USART_SERVO_R) {
		XM_RX_buffer_R.lastByteLength = bytes;
		XM_PORT_SERVO_R.OUTCLR = XM_OE_MASK;
	}

	// Send data
	while (!USART_IsTXDataRegisterEmpty(usart_data->usart))
		;
	for (i = 0; i < bytes; i++) {
		USART_PutChar(usart_data->usart, txData[i]);
		while (!USART_IsTXDataRegisterEmpty(usart_data->usart))
			;
	}

	// Enable TXC interrupt to set OE to 1
	USART_TxdInterruptLevel_Set(usart_data->usart, USART_TXCINTLVL_LO_gc);

}

/**
 * \brief 	USART-Empfangsmethode.
 *
 * 			Diese Methode liest den jeweiligen USART-Buffer aus und prüft,
 * 			ob ein vollständiges Paket gemäß des Dynamixel-Protokoll empfangen wurde.
 *
 * \param	rxBuffer	Empfangs-Buffer der jeweiligen USART
 * \param	dest		Byte-Array für Antwort-Paket
 *
 * \return	Länge des Antwortpakets
 */
DT_byte XM_USART_receive(DT_rxBuffer* rxBuffer, DT_byte* dest) {
	DEBUG_BYTE((rxBuffer->buffer, DT_RX_BUFFER_SIZE))
	// Cut off output message
	if (rxBuffer->lastByteLength > 0) {
		if ((rxBuffer->getIndex + rxBuffer->lastByteLength) < DT_RX_BUFFER_SIZE)
			rxBuffer->getIndex += rxBuffer->lastByteLength;
		else {
			rxBuffer->getIndex = rxBuffer->lastByteLength + rxBuffer->getIndex
					- DT_RX_BUFFER_SIZE;
			rxBuffer->overflow_flag = 0x00;
		}
		rxBuffer->lastByteLength = 0;
	}
	// Check errors
	if ((rxBuffer->overflow_flag == 0x00) && (rxBuffer->putIndex
			< rxBuffer->getIndex)) {
		return 0;
	} else if ((rxBuffer->overflow_flag == 0x01) && (rxBuffer->putIndex
			> rxBuffer->getIndex)) {
		return 0;
	} else if ((rxBuffer->buffer[rxBuffer->getIndex] != 0xFF)
			&& (rxBuffer->buffer[rxBuffer->getIndex + 1] != 0xFF)) {
		return 0;
	}
	// Some data received. All data received if checksum is correct!
	else {
		// Calculate predicted length
		DT_byte length;
		if ((rxBuffer->getIndex + 3) < DT_RX_BUFFER_SIZE)
			length = rxBuffer->buffer[rxBuffer->getIndex + 3];
		else {
			length = rxBuffer->buffer[3 + rxBuffer->getIndex
					- DT_RX_BUFFER_SIZE];
		}
		// Complete length = (FF + FF + ID + LENGTH) + length
		length += 4;
		// Copy packet from buffer in destination array
		DT_byte i;
		for (i = 0; i < length; i++) {
			if ((rxBuffer->getIndex + i) < DT_RX_BUFFER_SIZE)
				dest[i] = rxBuffer->buffer[rxBuffer->getIndex + i];
			else
				dest[i] = rxBuffer->buffer[i + rxBuffer->getIndex
						- DT_RX_BUFFER_SIZE];
		}
		// If packet is complete, set new getIndex
		if (dest[length - 1] == DNX_getChecksum(dest, length)) {
			if ((rxBuffer->getIndex + length) < DT_RX_BUFFER_SIZE)
				rxBuffer->getIndex = rxBuffer->getIndex + length;
			else {
				rxBuffer->getIndex = length + rxBuffer->getIndex
						- DT_RX_BUFFER_SIZE;
				rxBuffer->overflow_flag = 0x00;
			}
			return length;
		} else {
			return 0;
		}
	}
}

/**
 * \brief 	ISR für abgeschlossenen Sendevorgang der USARTC0 (SERVO L).
 */
ISR( USARTC0_TXC_vect) {
	USART_TxdInterruptLevel_Set(&USARTC0, USART_TXCINTLVL_OFF_gc);

	DT_byte i = 0;
	for (i = 0; i < 30; i++)
		; // delay

	XM_PORT_SERVO_L.OUTSET = XM_OE_MASK;
}

/**
 * \brief 	ISR für Empfangsvorgang der USARTC0 (SERVO L).
 */
ISR( USARTC0_RXC_vect) {
	USART_RXComplete(&XM_servo_data_L);
	if (USART_RXBufferData_Available(&XM_servo_data_L)) {
		XM_RX_buffer_L.buffer[XM_RX_buffer_L.putIndex++]
				= USART_RXBuffer_GetByte(&XM_servo_data_L);
	}
	if (XM_RX_buffer_L.putIndex >= DT_RX_BUFFER_SIZE) {
		XM_RX_buffer_L.putIndex = 0;
		XM_RX_buffer_L.overflow_flag = 0x01;
	}
}

/**
 * \brief 	ISR für abgeschlossenen Sendevorgang der USARTD0 (SERVO R).
 */
ISR( USARTD0_TXC_vect) {
	USART_TxdInterruptLevel_Set(&USARTD0, USART_TXCINTLVL_OFF_gc);

	DT_byte i = 0;
	for (i = 0; i < 30; i++)
		; // delay

	XM_PORT_SERVO_R.OUTSET = XM_OE_MASK;
}

/**
 * \brief 	ISR für Empfangsvorgang der USARTD0 (SERVO R).
 */
ISR( USARTD0_RXC_vect) {
	USART_RXComplete(&XM_servo_data_R);
	if (USART_RXBufferData_Available(&XM_servo_data_R)) {
		XM_RX_buffer_R.buffer[XM_RX_buffer_R.putIndex++]
				= USART_RXBuffer_GetByte(&XM_servo_data_R);
	}
	if (XM_RX_buffer_R.putIndex >= DT_RX_BUFFER_SIZE) {
		XM_RX_buffer_R.putIndex = 0;
		XM_RX_buffer_R.overflow_flag = 0x01;
	}
}
