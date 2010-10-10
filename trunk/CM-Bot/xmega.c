/*
 * xmega.c
 *
 *  Created on: 30.09.2010
 *      Author: christof
 */

#include "include/xmega.h"
#include "include/dynamixel.h"
#include <stdlib.h>

void XM_init_cpu() {
	// TODO
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
}

void XM_init_dnx() {
	//Disable Interrupts
	cli();

	// Init buffer
	XM_RX_buffer_L.getIndex = 0;
	XM_RX_buffer_L.putIndex = 0;

	XM_RX_buffer_R.getIndex = 0;
	XM_RX_buffer_R.putIndex = 0;

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
	// USART_RxdInterruptLevel_Set(XM_servo_data_R.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(XM_servo_data_L.usart, USART_RXCINTLVL_LO_gc);

	// Set Baudrate
	USART_Baudrate_Set(XM_servo_data_R.usart, 34, 0); // 57.600bps (BSEL = 34)
	USART_Baudrate_Set(XM_servo_data_L.usart, 1, 0); // 57.600bps (BSEL = 34)

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

void XM_init_com() {
	// TODO
}

void XM_USART_send(USART_data_t* usart_data, byte* txData, byte bytes) {
	byte i;

	// FIXME
	XM_RX_buffer_L.lastByteLength = bytes;

	if (usart_data->usart == &XM_USART_DEBUG)
		return;

	// Set OE to 0
	if (usart_data->usart == &XM_USART_SERVO_L) {
		XM_PORT_SERVO_L.OUTCLR = XM_OE_MASK;
	}
	if (usart_data->usart == &XM_USART_SERVO_R) {
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

	/*
	 while(XM_RX_buffer_L.putIndex < XM_RX_buffer_L.getIndex)
	 ;
	 */

}

byte XM_USART_receive(RXBuffer* rxBuffer, byte* dest) {
	// no new data
	if ((rxBuffer->putIndex <= rxBuffer->getIndex) && (rxBuffer->overflow_flag
			== 0))
		return -1;
	else if ((rxBuffer->putIndex >= rxBuffer->getIndex)
			&& (rxBuffer->overflow_flag == 1))
		return -1;
	else if ((rxBuffer->buffer[rxBuffer->getIndex] != 0xFF)
			&& (rxBuffer->buffer[rxBuffer->getIndex + 1] != 0xFF))
		return -1;
	else {
		byte length;
		if ((rxBuffer->getIndex + 3) < XM_RX_BUFFER_SIZE)
			length = rxBuffer->buffer[rxBuffer->getIndex + 3];
		else
			length = rxBuffer->buffer[2 + rxBuffer->getIndex
					- XM_RX_BUFFER_SIZE]; // 3 - 1, array 0 ... n-1;
		byte i;
		for (i = 0; i < length + 2; i++) {
			if ((rxBuffer->getIndex + i) < XM_RX_BUFFER_SIZE)
				dest[i] = rxBuffer->buffer[rxBuffer->getIndex + i];
			else
				dest[i] = rxBuffer->buffer[i - 1 + rxBuffer->getIndex
						- XM_RX_BUFFER_SIZE];
		}
		if (dest[length + 1] == DNX_getChecksum(dest, length + 1)) {

			return length + 2;
		}
		else //ToDo Nachricht komplett aber Checksum falsch (evtl. Timeout)
			return -1;
	}
}

/*
 ISR(USARTC0_DRE_vect) {
 // Send IRQ
 if (XM_sendCurrentCount < XM_sendCount) // Alles gesendet?
 USART_PutChar(&USARTC0, XM_sendBuffer[XM_sendCurrentCount++]); // TX Buffer fuellen

 else {
 USART_DreInterruptLevel_Set(&USARTC0, USART_DREINTLVL_OFF_gc); // DRE-IRQ sperren
 USART_TxdInterruptLevel_Set(&USARTC0, USART_TXCINTLVL_LO_gc); // Trans-IRO scharf machen
 }
 }
 */

ISR(USARTC0_TXC_vect)
{
	USART_TxdInterruptLevel_Set(&USARTC0, USART_TXCINTLVL_OFF_gc);

	byte i = 0;
	for (i = 0; i < 50; i++)
		; // delay

	XM_PORT_SERVO_L.OUTSET = XM_OE_MASK;
	XM_RX_buffer_L.putIndex = XM_RX_buffer_L.putIndex
			- XM_RX_buffer_L.lastByteLength;
}

ISR(USARTC0_RXC_vect)
{
	// TODO
	USART_RXComplete(&XM_servo_data_L);
	if (XM_RX_buffer_L.putIndex >= XM_RX_BUFFER_SIZE)
		XM_RX_buffer_L.putIndex = 0;
	if (USART_RXBufferData_Available(&XM_servo_data_L)) {
		XM_RX_buffer_L.buffer[XM_RX_buffer_L.putIndex++]
				= USART_RXBuffer_GetByte(&XM_servo_data_L);
	}
}

