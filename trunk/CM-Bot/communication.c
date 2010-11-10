/**
 * \file	communication.c
 *
 * \brief	Methoden zur Kommunikation der CPUs.
 */

#include "include/communication.h"
#include "include/utils.h"
#include "include/xmega.h"

#define COM_START_BYTE 	0xFF

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
DT_byte COM_getCpuID(DT_leg* leg_l) {
	if (leg_l->hip.id == 10)
		return COM_MASTER; // Master
	else if (leg_l->hip.id == 4)
		return COM_SLAVE3; // Slave
	else if (leg_l->hip.id == 16)
		return COM_SLAVE1; // Slave
	else
		return COM_NOCPUID; // falsche ID
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
	//DEBUG_BYTE((buffer,127))

	// Sind Daten vorhanden
	if (!USART_RXBufferData_Available(usart_data)) {
		DEBUG(("COMnd",sizeof("COMnd")))
		return 0;
	}
	// Init-Fehlerbytes ausfiltern
	else if (buffer->RX[buffer->RX_Tail] != 0xFF) {
		DEBUG(("COM_i",sizeof("COM_i")))
		USART_RXBuffer_GetByte(usart_data);
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
DT_byte COM_send(DT_byte* const packet, DT_size l, DT_byte* const result,
		const DT_bool response) {
	packet[l - 1] = COM_getChecksum(packet, l);
	USART_data_t* usart_data = &XM_com_data;
	// packet[2] -> ID
	XM_USART_send(usart_data, packet, l);

	DT_byte len = 0;
	DT_size timeout = response ? 100 : 0;

	while (len == 0 && timeout > 0) {
		len = COM_receive(usart_data, result);
		timeout--;
	}

	return len;
}

DT_size COM_requestStatus(DT_byte CpuID, DT_byte param, DT_byte* result) {
	// Broadcast bei requestStatus nicht möglich
	if (CpuID == COM_BRDCAST_ID)
		return 0;
	DT_size len = 7;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = CpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_STATUS;
	packet[5] = param;
	// packet[6] = checksum will set in send
	return COM_send(packet, len, result, true);

}

void COM_doubleToByteArray(const DT_double value, DT_byte* const array) {
	DT_byte* ptr = (DT_byte*) &value;
	for (DT_size i = 0; i < sizeof(DT_double); i++)
		array[i] = ptr[i];
}

DT_double COM_byteArrayToDouble(const DT_byte* const array) {
	DT_double value;
	DT_byte* ptr = (DT_byte*) &value;
	for (DT_size i = 0; i < sizeof(DT_double); i++)
		ptr[i] = array[i];
	return value;
}

DT_bool COM_sendPoint(DT_byte CpuID, DT_point* point) {
	// Broadcast bei requestStatus nicht möglich
	if (CpuID == COM_BRDCAST_ID)
		return 0;
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 18;
	DT_byte packet[len];

	// Point auf ByteArray casten
	COM_doubleToByteArray(point->x, &packet[5 + 0 * sizeof(DT_double)]);
	COM_doubleToByteArray(point->y, &packet[5 + 1 * sizeof(DT_double)]);
	COM_doubleToByteArray(point->z, &packet[5 + 2 * sizeof(DT_double)]);

	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = CpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_POINT;

	// packet[17] = checksum will set in send

	len = COM_send(packet, len, result, true);
	if ((len > 0) && (result[4] == COM_ACK))
		return true;
	else
		return false;
}

DT_point COM_getPointFromPaket(DT_byte* result) {
	DT_point p;

	p.x = COM_byteArrayToDouble(&result[5 + 0 * sizeof(DT_double)]);
	p.y = COM_byteArrayToDouble(&result[5 + 1 * sizeof(DT_double)]);
	p.z = COM_byteArrayToDouble(&result[5 + 1 * sizeof(DT_double)]);

	return p;
}

void COM_sendAction(DT_byte CpuID) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 6;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = CpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_ACTION;
	// packet[5] = checksum will set in send
	COM_send(packet, len, result, false);
}

DT_bool COM_isAlive(DT_byte CpuID) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len;
	len = COM_requestStatus(CpuID, COM_IS_ALIVE, result);
	if ((len > 0) && (result[4] == COM_ACK))
		return true;
	else
		return false;
}

void COM_sendACK(DT_byte CpuID) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 6;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = CpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_ACK;
	// packet[5] = checksum will set in send
	COM_send(packet, len, result, false);
}

void COM_sendNAK(DT_byte CpuID, DT_byte ErrCode) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 7;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = CpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_NAK;
	packet[5] = ErrCode;
	// packet[6] = checksum will set in send
	COM_send(packet, len, result, false);
}
