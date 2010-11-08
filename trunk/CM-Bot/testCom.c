/**
 * \file	testCom.c
 *
 * \brief	Testprogramm für Kommunikation der CPUs.
 */

#define TEST_ON
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/communication.h"

int main() {
	XM_init_cpu();
	XM_init_dnx();
	XM_init_com();
	XM_init_remote();

	XM_LED_ON

	DT_leg leg_r, leg_l;
	DT_byte CpuID;

	DNX_getConnectedIDs(&leg_r, &leg_l);
	CpuID = COM_getCpuID(&leg_r);

	switch (CpuID) {
	case MASTER:
		DEBUG(("Master",sizeof("Master")))
		break;
	case SLAVE1:
		DEBUG(("Slave1",sizeof("Slave1")))
		break;
	case SLAVE2:
		DEBUG(("Slave2",sizeof("Slave2")))
		break;
	default: //case NOCPUID:
		DEBUG(("NoCpuID",sizeof("NoCpuID")))
		break;
	}

	DT_size bufSize = 4;
	DT_byte txDataBuf[bufSize];
	txDataBuf[0] = 0xFF;
	txDataBuf[1] = 0x55;
	txDataBuf[2] = 0xAA;
	txDataBuf[3] = 0x12;
	DT_byte rxData[bufSize], l;

	while (1) {
		l = 0;
		switch (CpuID) {
		case MASTER:
			XM_USART_send(&XM_com_data, txDataBuf, bufSize);
			break;

		case SLAVE1:
			while (USART_RXBufferData_Available(&XM_com_data)) {
				rxData[l] = USART_RXBuffer_GetByte(&XM_com_data);
			}
			//COM_receive(&XM_com_data, rxData);
			DEBUG_BYTE((rxData,bufSize))
			break;

		case SLAVE2:
			while (USART_RXBufferData_Available(&XM_com_data)) {
				rxData[l] = USART_RXBuffer_GetByte(&XM_com_data);
			}
			//COM_receive(&XM_com_data, rxData);
			DEBUG_BYTE((rxData,bufSize))
			break;

		default: //case NOCPUID:
			DEBUG(("NoCpuID",sizeof("NoCpuID")))
			break;
		}
	}
	XM_LED_OFF

	return 0;
}

#endif /* TEST_ON */
