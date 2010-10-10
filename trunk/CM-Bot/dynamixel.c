/*
 * dynamixel.c
 *
 * Created on: 27.09.2010
 * Author: christof
 */

#include "include/dynamixel.h"
#include "include/xmega.h"

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

void DNX_send(byte* packet, byte l) {
	packet[l - 1] = DNX_getChecksum(packet, l);

	DEBUG_BYTE((packet, l))
	// FIXME
	XM_USART_send(&XM_servo_data_L, packet, l);
	/*
	 if (packet[2] < 4) { // use right servo
	 XM_USART_Send(XM_USART_SERVO_L, packet, l);
	 } else { // use left servo
	 XM_USART_Send(XM_USART_SERVO_R, packet, l);
	 }
	 */
}

void DNX_receive(byte* packet) {
	// TODO
	;
}

void DNX_sendTest() {
	byte packet[8];
	packet[0] = 0xA1;
	packet[1] = 0xA2;
	packet[2] = 0xA3;
	packet[3] = 0xA4; // length
	packet[4] = 0xA5;
	packet[5] = 0xA6;
	packet[6] = 0xA7;
	// packet[7] = checksum will set in send
	DNX_send(packet, 8);
}

void DNX_setAngle(byte id, byte value) {
	byte packet[9];
	byte angle=	value; // = calculate value
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x05; // length
	packet[4] = WR_DATA;
	packet[5] = GL_POS;
	// TODO high low
	packet[6] = angle; 	// Low
	packet[7] = 0x00;	// High
	// packet[7] = checksum will set in send
	DNX_send(packet, 9);
}

void DNX_setId(byte idOld, byte idNew) {
	byte packet[8];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = idOld;
	packet[3] = 0x04; // length
	packet[4] = WR_DATA;
	packet[5] = ID;
	packet[6] = idNew;
	// packet[7] = checksum will set in send
	DNX_send(packet, 8);

}

void DNX_setSpeed(byte id, byte speed) {
	byte packet[9];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x05; // length
	packet[4] = WR_DATA;
	packet[5] = MV_SPEED;
	packet[6] = speed;
	packet[7] = 0x00;
	// packet[7] = checksum will set in send
	DNX_send(packet, 9);
}

void DNX_setLed(byte id, byte value) {
	byte packet[8];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x04; // length
	packet[4] = WR_DATA;
	packet[5] = LED;
	packet[6] = value;
	// packet[7] = checksum will set in send
	DNX_send(packet, 8);
}

double DNX_getAngle(byte id) {
	// TODO
	byte packet[7];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x03; // length
	packet[4] = RD_DATA;
	packet[5] = PRT_POS;
	// packet[6] = checksum will set in send
	DNX_send(packet, 7);
	// TODO DNX_receive();
	return -1;
}

byte DNX_getSpeed(byte id) {
	byte packet[7];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x03; // length
	packet[4] = RD_DATA;
	packet[5] = PRT_SPEED;
	// packet[6] = checksum will set in send
	DNX_send(packet, 7);
	// TODO DNX_receive();
	return 0x00;
}

byte DNX_getLed(byte id) {
	byte packet[7];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = 0x03; // length
	packet[4] = RD_DATA;
	packet[5] = LED;
	// packet[6] = checksum will set in send
	DNX_send(packet, 7);
	// TODO DNX_receive();
	return 0x00;
}
