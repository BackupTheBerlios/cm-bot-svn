/**
 * \file	communication.h
 *
 * \brief	TODO
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "datatypes.h"
#include "usart_driver.h"

#define COM_MASTER		0x02
#define COM_SLAVE1		0x01
#define COM_SLAVE3		0x03
#define COM_BRDCAST_ID 	0xFE
#define COM_NOCPUID		0x00

// Instructions
#define COM_STATUS		0x01
#define COM_ACTION		0x02
#define COM_POINT		0x03

// Status Parameter
#define COM_IS_ALIVE	0x01

// Responses
#define COM_ACK			0x06
#define COM_NAK			0x15

// NAK-Error-Codes
#define COM_ERR_ANGLE_LIMIT			0x01
#define COM_ERR_POINT_OUT_OF_BOUNDS	0x02
#define COM_ERR_DEFAULT_ERROR		0x03

DT_byte COM_send(DT_byte* const, DT_size, DT_byte* const, DT_bool);
DT_byte COM_receive(USART_data_t* const, DT_byte* const);

DT_size COM_requestStatus(DT_byte, DT_byte, DT_byte* const);
DT_bool COM_sendPoint(DT_byte, const DT_point* const);
void COM_sendAction(DT_byte);
DT_bool COM_isAlive(DT_byte);
void COM_sendACK(DT_byte);
void COM_sendNAK(DT_byte, DT_byte);

DT_byte COM_getCpuID(const DT_leg* const);
DT_double COM_byteArrayToDouble(const DT_byte* const);
void COM_doubleToByteArray(const DT_double, DT_byte* const);
DT_point COM_getPointFromPaket(const DT_byte* const);

#endif /* COMMUNICATION_H_ */
