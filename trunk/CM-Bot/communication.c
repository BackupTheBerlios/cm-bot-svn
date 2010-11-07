/**
 * \file	communication.c
 *
 * \brief	Methoden zur Kommunikation der CPUs.
 */

#include "include/communication.h"
#include "include/utils.h"
#include "include/xmega.h"


/**
 * \brief	Berechnet die Checksum.
 *
 * \param	packet	Paket
 * \param	l	Größe des pakets
 *
 * \return	Checksum
 */
DT_byte COM_getChecksum(const DT_byte* const packet, DT_size l) {
	DT_size i;
	DT_byte chksm = 0;
	for (i = 2; i < l - 1; i++)
		chksm += packet[i];
	return ~chksm;
}


/*
 *  \brief 	Zuweisung der CpuID.
 *
 * 			Achtung: vorher muss die Methode DNX_getConnectedIDs(leg_r, leg_l) ausgeführt werden,
 * 			damit in dem Bein-Struct die IDs der angeschlossenen Geräte stehen.
 *
 * \param	leg_r		Bein-Struktur mit IDs
 *
 * \return	ID	1->Master; 2,3->Slave; 0 Error
 */
DT_byte COM_getCpuID(DT_leg* leg_r) {
	if (leg_r->hip.id == 7)
		return 1; // Master
	else if (leg_r->hip.id == 1)
		return 2; // Slave
	else if (leg_r->hip.id == 13)
		return 3; // Slave
	else
		return 0; // falsche ID
}

/**ToDo
 * \brief 	USART-Empfangsmethode für Prozessorkommunikation.
 *
 * 			Diese Methode liest den jeweiligen USART-Buffer aus und prüft,
 * 			ob ein vollständiges Paket gemäß des Communication-Protokoll empfangen wurde.
 *
 * \param	usart_data	USART-Datenstruktur
 * \param	dest		Byte-Array für Antwort-Paket
 *
 * \return	Länge des Antwortpakets
 */
DT_byte COM_receive(USART_data_t* const usart_data, DT_byte* const dest) {
	USART_Buffer_t* buffer = &usart_data->buffer;

	while (USART_RXBufferData_Available(usart_data)
			&& usart_data->lastPacketLength > 0) {
		usart_data->lastPacketLength--;
		USART_RXBuffer_GetByte(usart_data);
	}
	if (usart_data->lastPacketLength > 0) {
		DEBUG(("COMse",sizeof("COMse")))
		return 0;
	}

	// Sind Daten vorhanden
	if (!USART_RXBufferData_Available(usart_data)) {
		DEBUG(("COMnd",sizeof("COMnd")))
		return 0;
	}
	// Byte #1 und Byte #2 muessen laut Protokoll 0xFF sein
	else if ((buffer->RX[buffer->RX_Tail] != 0xFF)
			&& (buffer->RX[(buffer->RX_Tail + 1) & USART_RX_BUFFER_MASK]
					!= 0xFF)) {
		DEBUG(("COMff",sizeof("COMff")))
		return 0;
	}
	// Pruefen ob min. 4 Bytes im Buffer sind, um Laenge zu lesen
	else if ((buffer->RX_Head > buffer->RX_Tail) && (buffer->RX_Head
			- buffer->RX_Tail) < 4) {
		DEBUG(("COMle",sizeof("COMle")))
		return 0;
	} else if ((buffer->RX_Head < buffer->RX_Tail) && (USART_RX_BUFFER_SIZE
			- buffer->RX_Tail + buffer->RX_Head) < 4) {
		DEBUG(("COMle",sizeof("COMle")))
		return 0;
	}
	// Some data received. All data received if checksum is correct!
	else {
		// Calculate predicted length
		DT_byte length;
		length = buffer->RX[(buffer->RX_Tail + 3) & USART_RX_BUFFER_MASK];
		// Complete length = (FF + FF + ID + LENGTH) + length
		length += 4;
		// Prüfen ob Paket bereits komplett im Buffer
		if (((buffer->RX_Tail + length) & USART_RX_BUFFER_MASK)
				> buffer->RX_Head) {
			DEBUG(("uc",sizeof("uc")))
			return 0;
		}
		// Copy packet from buffer in destination array
		DT_byte i;
		for (i = 0; i < length; i++) {
			dest[i] = USART_RXBuffer_GetByte(usart_data);
		}

		// Pruefen ob Checksumme korrekt ist
		// ToDo: im Fehlerfall Buffer zurücksetzen
		if (dest[length - 1] != COM_getChecksum(dest, length)) {
			DEBUG(("cks",sizeof("cks")))
			DEBUG_BYTE((dest, length))
			return 0;
		}
		DEBUG(("ok",sizeof("ok")))
		return length;
	}
}

/**
 * \brief	Versenden von Daten an anderen Controller.
 *
 * 			Blockierendes Senden mit gleichzeitigem Empfangen der Antwort.
 *
 * \param	packet	Zuversendendes Paket
 * \param	l	Größe des Pakets
 * \param	result	Zielfeld für Antowort
 *
 * \return Größe der empfangenen Antwort
 */
DT_byte COM_send(DT_byte* const packet, DT_size l, DT_byte* const result) {
	packet[l - 1] = COM_getChecksum(packet, l);
	USART_data_t* usart_data = &XM_com_data;
	// packet[2] -> ID

	XM_USART_send(usart_data, packet, l);

	DT_byte len = 0;
	DT_size timeout = 100;
	while (len == 0 && timeout > 0) {
		len = COM_receive(usart_data, result);
		timeout--;
	}

	return len;
}
