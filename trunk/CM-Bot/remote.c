/**
 * \file	remote.c
 *
 * \brief	Methoden zur Steuerung durch den RC-100 Remote Controller.
 */

#include "include/remote.h"
#include "include/xmega.h"
#include "include/utils.h"

/* Instruction aus dem High- und Low-Teil zusammensetzen
 * Bsp:
 * 	Paket für B_2:
 *      0  1  2  3  4  5  ...
 * 		FF;55;20;DF;00;FF;FF;55;00;FF;00;FF;
 *            LL    HH
 * 	Eigentliche Information:
 * 		0x0020
 *  d.h.
 *      H = Paket[4]
 *      L = Paket[2]
 */
DT_cmd RMT_getInstruction() {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_cmd cmd = 0x0000, tmp_cmd = 0xFFFF;
	DT_bool button_release = false;
	RMT_receive(&XM_remote_data, result);
	cmd = (result[4] << 8) | result[2];
	while (!button_release) {
		//remoteReceive(result);
		tmp_cmd = (result[4] << 8) | result[2];
		if (tmp_cmd == B_NON_PRESSED)
			button_release = true;
	}
	return cmd;
}

/*
 * \brief 	USART-Empfangsmethode für Remote-Controller.
 *
 * 			Diese Methode liest den Remote-USART-Buffer aus und prüft,
 * 			ob ein vollständiges Paket empfangen wurde.
 *
 * \param	rxBuffer	Empfangs-Buffer der jeweiligen USART
 * \param	dest		Byte-Array für Antwort-Paket
 *
 * \return	Länge des Antwortpakets
 */
DT_byte RMT_receive(USART_data_t* const usart_data, DT_byte* const dest) {
	USART_Buffer_t* buffer = &usart_data->buffer;

	// Sind Daten vorhanden
	if (!USART_RXBufferData_Available(usart_data)) {
		DEBUG(("RMT_nd",sizeof("RMT_nd")))
		return 0;
	}
	// Byte #1 und Byte #2 muessen laut Protokoll 0xFF und 0x55 sein
	else if ((buffer->RX[buffer->RX_Tail] != 0xFF)
			&& (buffer->RX[(buffer->RX_Tail + 1) & USART_RX_BUFFER_MASK]
					!= 0x55)) {
		DEBUG(("RMT_ff",sizeof("RMT_ff")))
		return 0;
	}
	// Some data received. All data received if checksum is correct!
	else {
		// length = (FF + 55 + LL + !LL + HH + !HH)
		DT_byte length = 6;
		// Prüfen ob Paket bereits komplett im Buffer
		if (((buffer->RX_Tail + length) & USART_RX_BUFFER_MASK)
				> buffer->RX_Head) {
			DEBUG(("RMT_uc",sizeof("RMT_uc")))
			return 0;
		}
		// Copy packet from buffer in destination array
		DT_byte i;
		for (i = 0; i < length; i++) {
			dest[i] = USART_RXBuffer_GetByte(usart_data);
		}

		DEBUG(("RMT_ok",sizeof("RMT_ok")))
		return length;
	}
}

#ifdef todo
/*
 * /**
 * \brief 	USART-Empfangsmethode für Remote-Controller.
 *
 * 			Diese Methode liest den Remote-USART-Buffer aus und prüft,
 * 			ob ein vollständiges Paket empfangen wurde.
 *
 * \param	rxBuffer	Empfangs-Buffer der jeweiligen USART
 * \param	dest		Byte-Array für Antwort-Paket
 *
 * \return	Länge des Antwortpakets
 */
DT_byte XM_REMOTE_USART_receive(DT_rxBuffer* const rxBuffer,
		DT_byte* const dest) {
	//DEBUG_BYTE((rxBuffer->buffer, DT_RX_BUFFER_SIZE))
	// Cut off output message
	if (rxBuffer->lastPacketLength > 0) {
		if ((rxBuffer->getIndex + rxBuffer->lastPacketLength)
				< DT_RX_BUFFER_SIZE)
		rxBuffer->getIndex += rxBuffer->lastPacketLength;
		else {
			rxBuffer->getIndex = rxBuffer->lastPacketLength
			+ rxBuffer->getIndex - DT_RX_BUFFER_SIZE;
			rxBuffer->overflow_flag = 0x00;
		}
		rxBuffer->lastPacketLength = 0;
	}
	// Check errors
	if ((rxBuffer->overflow_flag == 0x00) && (rxBuffer->putIndex
					< rxBuffer->getIndex)) {
		return 0;
	} else if ((rxBuffer->overflow_flag == 0x01) && (rxBuffer->putIndex
					> rxBuffer->getIndex)) {
		return 0;
	} else if ((rxBuffer->buffer[rxBuffer->getIndex] != 0xFF)
			&& rxBuffer->buffer[rxBuffer->getIndex + 1] != 0x55) {
		return 0;
	}
	// Some data received.
	else {
		DT_byte length;
		// length = (FF + 55 + LL + !LL + HH + !HH)
		length = 6;
		// Copy packet from buffer in destination array
		DT_byte i;
		if (rxBuffer->getIndex + length <= rxBuffer->putIndex) {
			for (i = 0; i < length; i++) {
				if ((rxBuffer->getIndex + i) < DT_RX_BUFFER_SIZE)
				dest[i] = rxBuffer->buffer[rxBuffer->getIndex + i];
				else
				dest[i] = rxBuffer->buffer[i + rxBuffer->getIndex
				- DT_RX_BUFFER_SIZE];
			}
			// If packet is complete, set new getIndex
			if ((rxBuffer->getIndex + length) < DT_RX_BUFFER_SIZE)
			rxBuffer->getIndex = rxBuffer->getIndex + length;
			else {
				rxBuffer->getIndex = length + rxBuffer->getIndex
				- DT_RX_BUFFER_SIZE;
				rxBuffer->overflow_flag = 0x00;
			}
			return length;
		} else
		return 0;
	}
}

#endif
