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
DT_cmd getInstruction() {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_cmd cmd = 0x0000, tmp_cmd = 0xFFFF;
	DT_byte button_release = 0x00, timeout = 0xFF;
	remoteReceive(result);
	cmd = (result[4] << 8) | result[2];
	while (!button_release == 0x01) {
		remoteReceive(result);
		tmp_cmd = (result[4] << 8) | result[2];
		if (tmp_cmd == B_NON_PRESSED || timeout == 0x00)
			button_release = 0x01;
	}
	return cmd;
}

DT_byte remoteReceive(DT_byte* result) {
	DT_byte len = 0;

	while (len == 0) {
		//len = (&XM_RX_remote, result);
	}
	//DEBUG_BYTE((result, len))
	return len;
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
		if ((rxBuffer->getIndex + rxBuffer->lastPacketLength) < DT_RX_BUFFER_SIZE)
			rxBuffer->getIndex += rxBuffer->lastPacketLength;
		else {
			rxBuffer->getIndex = rxBuffer->lastPacketLength + rxBuffer->getIndex
					- DT_RX_BUFFER_SIZE;
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
