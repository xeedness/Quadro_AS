/*
 * uart_bridge.c
 *
 * Created: 14.06.2020 20:26:34
 *  Author: xeedn
 */ 

#include <asf.h>


#include <string.h>
#include "uart_bridge.h"

#define SEND_BUF_LENGTH (128)

void uart_bridge_init(void) {
	const usart_serial_options_t usart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = US_MR_PAR_NO,
		.stopbits = US_MR_NBSTOP_1_BIT,
		.charlength = US_MR_CHRL_8_BIT
	};
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_init((Usart *)USART0,(usart_serial_options_t *)(&usart_serial_options));
	
	// Higher priority for control signals ?
	/*usart_enable_interrupt(CONF_UART_CONTROL, UART_IER_RXRDY);
	NVIC_SetPriority(USART0_IRQn, 1);
	NVIC_EnableIRQ(USART0_IRQn);*/
}

static bool uart_bridge_send_wrapped(uint8_t* data, size_t length)
{
	uint8_t buf[SEND_BUF_LENGTH + 2] = { 0 };
	
	buf[0] = ASCII_STX;
	memcpy(buf+1, data, length);
	buf[length+1] = ASCII_ETX;
	
	uart_bridge_send_raw(buf, length + 2);
	
	return true;
}

bool uart_bridge_send_request(const uint8_t type) {
	return uart_bridge_send(&type, 1);
}

void uart_bridge_send_raw(const uint8_t* data, size_t size) {
	usart_serial_write_packet((Usart *)USART0, data, size);
}

/*ISR(USART0_Handler)
{	
	static uint8_t value, rcv_buf[256], dec_buf[256];
	static size_t offset = 0;
	static bool transmission_active = false;
	
	usart_serial_read_packet(USART0, &value, 1);
	//printf("%2X %i\n", value, offset);
	
	if(value == ASCII_STX)
	{
		offset = 0;
		transmission_active = true;
	}
	else if(transmission_active)
	{
		if(value == ASCII_ETX)
		{
			decode_85(dec_buf, rcv_buf, offset);
			execute_command(dec_buf[0], dec_buf+1);
			offset = 0;
			transmission_active = false;
		}
		else
		{
			rcv_buf[offset++] = value;
			offset %= 256;
		}
	}
	
	//uint32_t dummy = usart_get_interrupt_mask(USART0);
}*/

