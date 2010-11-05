/*
 * remote.h
 *
 *  Created on: 29.10.2010
 *      Author: ricky
 */

#ifndef REMOTE_H_
#define REMOTE_H_

#include "datatypes.h"
#include "usart_driver.h"

// Commands
#define B_NON_PRESSED 0x0000

#define B_U 0x0001
#define B_D 0x0002
#define B_L 0x0004
#define B_R 0x0008

#define B_1 0x0010
#define B_2 0x0020
#define B_3 0x0040
#define B_4 0x0080
#define B_5 0x0100
#define B_6 0x0200


DT_cmd RMT_getInstruction();
DT_byte RMT_receive(USART_data_t* const, DT_byte* const);

#endif /* REMOTE_H_ */
