/*
 * dynamixel.c
 *
 * Created on: 27.09.2010
 * Author: christof
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

byte DNX_getChecksum(byte* packet, byte l) {
	byte i, chksm = 0;
	for (i = 2; i < l - 1; i++)
		chksm += packet[i];
	return ~chksm;
}

int DNX_receive(byte id, byte* result) {
	int len = 0;
	RXBuffer* rxBuffer;

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

int DNX_send(byte* packet, byte l, byte* result) {
	packet[l - 1] = DNX_getChecksum(packet, l);
	// packet[2] -> ID
	if ((packet[2] - 1) % 6 < 3) // Right: 1 - 3, 7 - 9, ...
		XM_USART_send(&XM_servo_data_R, packet, l);
	else
		// Left:  4 - 6, ...
		XM_USART_send(&XM_servo_data_L, packet, l);
	return DNX_receive(packet[2], result);
}

void DNX_sendTest() {
	byte result[XM_RESULT_BUFFER_SIZE];
	byte packet[8];
	packet[0] = 0xA1;
	packet[1] = 0xA2;
	packet[2] = 0xA3;
	packet[3] = 0xA4; // length
	packet[4] = 0xA5;
	packet[5] = 0xA6;
	packet[6] = 0xA7;
	// packet[7] = checksum will set in send
	DNX_send(packet, 8, result);
}

double DNX_convertAngle(double value) {
	value += 150;
	if (value >= 360)
		value -= 360;
	return value;
}

double correctAngles(byte id, double value){
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

void DNX_setAngle(byte id, double value) {
	//ToDO
	byte result[XM_RESULT_BUFFER_SIZE];
	byte packet[9];
	value = correctAngles(id, value);
	value = DNX_convertAngle(value);
	double tmp2 = ((double) value) * 3.41;
	int tmp = floor(tmp2);
	byte angle_l = tmp & 0xFF;
	byte angle_h = tmp >> 8;
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

void DNX_setId(byte idOld, byte idNew) {
	byte packet[8];
	byte result[XM_RESULT_BUFFER_SIZE];
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

void DNX_setSpeed(byte id, byte speed) {
	byte packet[9];
	byte result[XM_RESULT_BUFFER_SIZE];
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

void DNX_setLed(byte id, byte value) {
	byte packet[8];
	byte result[XM_RESULT_BUFFER_SIZE];
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

double DNX_getAngle(byte id) {
	// TODO
	byte packet[7];
	byte result[XM_RESULT_BUFFER_SIZE];
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

byte DNX_getSpeed(byte id) {
	byte packet[7];
	byte result[XM_RESULT_BUFFER_SIZE];
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

byte DNX_getLed(byte id) {
	byte packet[7];
	byte result[XM_RESULT_BUFFER_SIZE];
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
