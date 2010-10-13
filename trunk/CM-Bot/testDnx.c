/*
 * testDnx.c
 *
 * Created on: 29.09.2010
 * Author: christof
 */

#include "include/xmega.h"
#include "include/utils.h"
#include "include/dynamixel.h"

#define TEST_ON
#ifdef TEST_ON

int main() {
	XM_init_cpu();
	XM_init_dnx();

	byte received_Data[XM_RX_BUFFER_SIZE];
	byte len = 0;
	uint16_t value = 50;
	byte led = 0x00;
#define _fdf
#ifdef _fdf
	while (1) {
		UTL_wait(2);
		DNX_setLed(0x06, led);

		while (len == 0) {
			len = XM_USART_receive(&XM_RX_buffer_L, received_Data);
		}

		if(led == 0x00) {
			led = 0x01;
			XM_LED_ON
		} else {
			led =  0x00;
			XM_LED_OFF
		}
	}
#endif
#define _asf_OFF
#ifdef _asf
	while (1) {
		//if(XM_USART_receive(&XM_RX_buffer_L, received_Data) != 0){
		//DEBUG_BYTE((XM_RX_buffer_L.buffer, XM_RX_buffer_L.putIndex))
		//XM_LED_ON
		//}
		//if (SWITCH_PRESSED) {
		UTL_wait(50);
		DNX_setAngle(0x06, value);
		//value += 50;
		if (value == 50)
			value = 250;
		else
			value = 50;
		//while(XM_USART_receive(&XM_RX_buffer_L, received_Data) == 0);
		//DEBUG_BYTE((XM_RX_buffer_L.buffer, XM_RX_buffer_L.putIndex))
		/*while (SWITCH_PRESSED)
		 XM_LED_ON;
		 XM_LED_OFF*/
		len = 0;
		XM_LED_OFF
		UTL_wait(50);
		/*
		 while (len == 0) {
		 len = XM_USART_receive(&XM_RX_buffer_L, received_Data);
		 }
		 */
		XM_LED_ON
		DEBUG_BYTE((received_Data, len))
		//}
	}
#endif
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
