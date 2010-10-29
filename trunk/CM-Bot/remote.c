/**
 * \file	remote.c
 *
 * \brief	Methoden zur Steuerung durch den RC-100 Remote Controller.
 */

#include "include/remote.h"

// ToDo
DT_byte getInstruction() {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_byte Instruction = 0;
	while (remoteReceive(result) == 0)
		;
	/* ToDo
	 * Instruction aus dem High- und Low-Teil zusammensetzen
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

	return Instruction;
}

// ToDo
DT_byte remoteReceive(DT_byte* result) {
	/*while (!USART_IsRXComplete(&XM_USART_REMOTE))
	 ;
	 receivedData = USART_GetChar(XM_remote_data.usart);
	 */

}
