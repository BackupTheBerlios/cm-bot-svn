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

DT_cmd RMT_getCommand();
DT_byte RMT_receive(USART_data_t* const, DT_byte* const);
DT_bool isUpPressed(DT_cmd);
DT_bool isDownPressed(DT_cmd);
DT_bool isLeftPressed(DT_cmd);
DT_bool isRightPressed(DT_cmd);
DT_bool isButton1Pressed(DT_cmd);
DT_bool isButton2Pressed(DT_cmd);
DT_bool isButton3Pressed(DT_cmd);
DT_bool isButton4Pressed(DT_cmd);
DT_bool isButton5Pressed(DT_cmd);
DT_bool isButton6Pressed(DT_cmd);

#endif /* REMOTE_H_ */
