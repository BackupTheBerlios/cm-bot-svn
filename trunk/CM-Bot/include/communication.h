/**
 * \file	communication.h
 *
 * \brief	TODO
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "datatypes.h"
#include "usart_driver.h"

DT_byte COM_getCpuID(DT_leg*);
DT_byte COM_receive(USART_data_t* const, DT_byte* const) ;
DT_byte COM_send(DT_byte* const, DT_size, DT_byte* const);

#endif /* COMMUNICATION_H_ */
