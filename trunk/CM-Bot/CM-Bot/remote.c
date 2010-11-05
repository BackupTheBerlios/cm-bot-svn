/**
 * \file	remote.c
 *
 * \brief	Methoden zur Steuerung durch den RC-100 Remote Controller.
 */

#include "include/remote.h"
#include "include/xmega.h"
#include "include/utils.h"

/* Instruction aus dem High- und Low-Teil zusammensetzen
 * Bsp:
 * 	Paket für B_2:
 *      0  1  2  3  4  5  ...
 * 		FF;55;20;DF;00;FF;FF;55;00;FF;00;FF;
 *            LL    HH
 * 	Eigentliche Information:
 * 		0x0020
 *  d.h.
 *      H = Paket[4]
 *      L = Paket[2]
 */
DT_cmd getInstruction() {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_cmd cmd = 0x0000, tmp_cmd = 0xFFFF;
	DT_byte button_release = 0x00, timeout = 0xFF;
	remoteReceive(result);
	cmd = (result[4] << 8) | result[2];
	while (!button_release == 0x01) {
		remoteReceive(result);
		tmp_cmd = (result[4] << 8) | result[2];
		if (tmp_cmd == B_NON_PRESSED || timeout == 0x00)
			button_release = 0x01;
	}
	return cmd;
}

DT_byte remoteReceive(DT_byte* result) {
	DT_byte len = 0;

	while (len == 0) {
		len = XM_REMOTE_USART_receive(&XM_RX_remote, result);
	}
	//DEBUG_BYTE((result, len))
	return len;
}

