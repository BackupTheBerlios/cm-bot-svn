/**
 * \file	dynamixel.c
 *
 * \brief	Methoden zur Steuerung der Dynamixal AX-12.
 */

#include "include/dynamixel.h"
#include "include/xmega.h"
#include <math.h>

#define START_BYTE 0xFF

// Instruction Set (Manual page 19)
#define PING 0x01
#define RD_DATA 0x02
#define WR_DATA 0x03
#define REG_WR 0x04
#define ACT 0x05
#define RESET 0x06
#define SYC_WR 0x83

// Control Table (Manuel page 12) L/H?
#define ID 0x03
#define BD 0x04
#define MAX_TMP 0x0B
#define STS_RT_LVL 0x10
#define ALR_SHUTDWN 0x12
#define GL_POS 0x1E
#define LED 0x19
#define MV_SPEED 0x20
#define PRT_POS 0x24
#define PRT_SPEED 0x26
#define PRT_TMP 0x2B

/**
 * \brief	Berechnet die Checksum.
 *
 * \param	packet	Paket
 * \param	l	Größe des pakets
 *
 * \return	Checksum
 */
DT_byte DNX_getChecksum(DT_byte* packet, DT_size l) {
	DT_size i;
	DT_byte chksm = 0;
	for (i = 2; i < l - 1; i++)
		chksm += packet[i];
	return ~chksm;
}

/**
 * \brief	Blockierendes Empfangen.
 *
 * \param	id	ID des Servos um linke/rechte Seite feststellen zu können
 * \param	result	Zielfeld für Daten
 *
 * \return	Größe der empfangenen Daten
 */
DT_size DNX_receive(DT_byte id, DT_byte* result) {
	DT_size len = 0;
	DT_rxBuffer* rxBuffer;

	// TODO Berechnung Auslagern ...
	if ((id - 1) % 6 < 3) { // Right: 1 - 3, 7 - 9, ...
		rxBuffer = &XM_RX_buffer_R;
	} else {
		// Left:  4 - 6, ...
		rxBuffer = &XM_RX_buffer_L;
	}

	while (len == 0) {
		len = XM_USART_receive(rxBuffer, result);
	}

	return len;
}

/**
 * \brief	Versenden von Daten an Dynamixel.
 *
 * 			Blockierendes Senden mit gleichzeitigem Empfangen der Antwort.
 *
 * \param	packet	Zuversendendes Paket
 * \param	l	Größe des Pakets
 * \param	result	Zielfeld für Antowort
 *
 * \return Größe der empfangenen Antwort
 */
DT_size DNX_send(DT_byte* packet, DT_size l, DT_byte* result) {
	packet[l - 1] = DNX_getChecksum(packet, l);
	// TODO Berechnung Auslagern ...
	// packet[2] -> ID
	if ((packet[2] - 1) % 6 < 3) // Right: 1 - 3, 7 - 9, ...
		XM_USART_send(&XM_servo_data_R, packet, l);
	else
		// Left:  4 - 6, ...
		XM_USART_send(&XM_servo_data_L, packet, l);
	return DNX_receive(packet[2], result);
}

/**
 * \brief	Konvertiert Winkel in Bezug auf einen neuen Nullpunkt.
 *
 * \param	value	Winkel in Grad
 *
 * \return	Konvertierter Winkel in Grad
 */
DT_double DNX_convertAngle(DT_double value) {
	value += 150;
	if (value >= 360)
		value -= 360;
	return value;
}


/**
 * \brief	Korrigiert Winkel für Dynamixel.
 *
 * 			Korrigiert Winkel für Dynamixel um hardwareseitige Veränderungen auszuschließen.
 *
 * \param	id	ID des Servos
 * \param	value	Winkel in Grad
 *
 * \return	Korrigierter Winkel in Grad
 */
DT_double DNX_correctAngles(DT_byte id, DT_double value){
	switch((id-1) % 6){
	case 0:
		value = 360-value;
		break;
	case 1:
		// value = value;
		break;
	case 2:
		value = 360-value;
		break;
	case 3:
		value = 360-value;
		break;
	case 4:
		value = 360-value;
		break;
	case 5:
		// value = value;
		break;
	}
	return value;
}

/**
 * \brief	Sendet einen sofort anzufahrenden Winkel an Servo.
 *
 * \param	id	ID des Servos
 * \param	value	Winkel in Grad
 */
void DNX_setAngle(DT_byte id, DT_double value) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_byte packet[9];
	value = DNX_correctAngles(id, value);
	value = DNX_convertAngle(value);

	// TODO Rechnung überarbeiten
	double tmp2 = ((double) value) * 3.41;
	int tmp = floor(tmp2);
	DT_byte angle_l = tmp & 0xFF;
	DT_byte angle_h = tmp >> 8;

	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x05; // length
	packet[4] = WR_DATA;
	packet[5] = GL_POS;
	packet[6] = angle_l; // Low
	packet[7] = angle_h; // High
	// packet[7] = checksum will set in send
	DNX_send(packet, 9, result);
}

/**
 * \brief	Vergibt einem Servos eine neue ID (ungetestet).
 *
 * \param	idOld	ID des zu verändernden Servos
 * \param	idNew	Zusetzende ID
 */
void DNX_setId(DT_byte idOld, DT_byte idNew) {
	DT_byte packet[8];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = idOld;
	packet[3] = 0x04; // length
	packet[4] = WR_DATA;
	packet[5] = ID;
	packet[6] = idNew;
	// packet[7] = checksum will set in send
	DNX_send(packet, 8, result);

}

/**
 * \brief	Setzt die Anfahrgeschwindigkeit eines Servos (unvollendet).
 *
 * \param	id	ID des Servos
 * \param	speed	Geschwindigkeit
 */
void DNX_setSpeed(DT_byte id, DT_byte speed) {
	// TODO byte 7 richtige setzen
	DT_byte packet[9];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x05; // length
	packet[4] = WR_DATA;
	packet[5] = MV_SPEED;
	packet[6] = speed;
	packet[7] = 0x00;
	// packet[7] = checksum will set in send
	DNX_send(packet, 9, result);
}

/**
 * \brief	Schaltet die LED eines Servos an/aus.
 *
 * \param	id	ID des Servos
 * \param	value	Wert für LED (0x00 / 0x01)
 */
void DNX_setLed(DT_byte id, DT_byte value) {
	DT_byte packet[8];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x04; // length
	packet[4] = WR_DATA;
	packet[5] = LED;
	packet[6] = value;
	// packet[7] = checksum will set in send
	DNX_send(packet, 8, result);
}

/**
 * \brief	Liest den aktuellen Winkel eines Servos aus (unfertig).
 *
 * \param	id	ID des Servos
 *
 * \return Winkel in Grad
 */
DT_double DNX_getAngle(DT_byte id) {
	// TODO
	DT_byte packet[7];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x03; // length
	packet[4] = RD_DATA;
	packet[5] = PRT_POS;
	// packet[6] = checksum will set in send
	DNX_send(packet, 7, result);
	// TODO DNX_receive();
	return -1;
}

/**
 * \brief	Liest die Anfahrgeschwindigkeit eines Servos aus (unfertig).
 *
 * \param	id	ID des Servos
 *
 * \return	Geschwindigkeit
 */
DT_byte DNX_getSpeed(DT_byte id) {
	DT_byte packet[7];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x03; // length
	packet[4] = RD_DATA;
	packet[5] = PRT_SPEED;
	// packet[6] = checksum will set in send
	DNX_send(packet, 7, result);
	// TODO DNX_receive();
	return 0x00;
}

/**
 * \brief	Liest den Status der LED aus (unfertig).
 *
 * \param	id	ID des Servos
 *
 * \return	Wert der LED
 */
DT_byte DNX_getLed(DT_byte id) {
	DT_byte packet[7];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x03; // length
	packet[4] = RD_DATA;
	packet[5] = LED;
	// packet[6] = checksum will set in send
	DNX_send(packet, 7, result);
	// TODO DNX_receive();
	return 0x00;
}
