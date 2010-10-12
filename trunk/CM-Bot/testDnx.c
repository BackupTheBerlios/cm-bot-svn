/*
 * testDnx.c
 *
 * Created on: 29.09.2010
 * Author: christof
 */

#include "include/xmega.h"
#include "include/utils.h"
#include "include/dynamixel.h"

#define TEST_OFF
#ifdef TEST_ON

int main() {
	XM_init_cpu();
	XM_init_dnx();

	DNX_setLed(0x06, 0x01);

	XM_LED_OFF
	byte received_Data[256];
	byte len = 0;
	int i = 0;
	int value;

	while (1) {
		//if(XM_USART_receive(&XM_RX_buffer_L, received_Data) != 0){
		//DEBUG_BYTE((XM_RX_buffer_L.buffer, XM_RX_buffer_L.putIndex))
		//XM_LED_ON
		//}
		if (SWITCH_PRESSED) {
			value = 50 + i;
			DNX_setAngle(0x06, value);
			i++;
			if (i >= 200)
				i = 0;
			//while(XM_USART_receive(&XM_RX_buffer_L, received_Data) == 0);
			//DEBUG_BYTE((XM_RX_buffer_L.buffer, XM_RX_buffer_L.putIndex))
			while (SWITCH_PRESSED)
				XM_LED_ON;
			XM_LED_OFF
			len = 0;
			while (len==0) {
				len = XM_USART_receive(&XM_RX_buffer_L, received_Data);
			}
			DEBUG_BYTE((received_Data, len))
		}
	}

	return 0;
}

/*
 ISR(USARTC0_RXC_vect) {
 USART_RXComplete(&XM_servo_data_L);
 if (USART_RXBufferData_Available(&XM_servo_data_L)) {
 // copy buffer to IRmsgRx
 XM_RX_buffer_L[0] = USART_RXBuffer_GetByte(&XM_servo_data_L);
 }
 }
 */

#endif /* TEST_ON */
