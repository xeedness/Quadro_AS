/*
 * uart_bridge.c
 *
 * Created: 14.06.2020 20:26:34
 *  Author: xeedn
 */ 

#include "uart_bridge.h"
#include <asf.h>
#include "config/conf_usart.h"

void setup_uart_bridge(void) {
	const usart_serial_options_t usart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = US_MR_PAR_NO,
		.stopbits = US_MR_NBSTOP_1_BIT,
		.charlength = US_MR_CHRL_8_BIT
	};
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_init((Usart *)CONF_UART_CONTROL,(usart_serial_options_t *)(&usart_serial_options));
	
	// TODO Enable interrupt and do sth with it
}

void uart_bridge_send(const uint8_t* data, size_t size) {
	usart_serial_write_packet((Usart *)CONF_UART_CONTROL, data, size);
}

