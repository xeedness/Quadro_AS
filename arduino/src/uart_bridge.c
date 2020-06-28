/*
 * uart_bridge.c
 *
 * Created: 14.06.2020 20:26:34
 *  Author: xeedn
 */ 

#include <asf.h>

#include "uart_bridge.h"
#include "config/conf_usart.h"
#include "base85.h"
#include "commander.h"

void setup_uart_bridge(void) {
	const usart_serial_options_t usart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = US_MR_PAR_NO,
		.stopbits = US_MR_NBSTOP_1_BIT,
		.charlength = US_MR_CHRL_8_BIT
	};
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_init((Usart *)CONF_UART_CONTROL,(usart_serial_options_t *)(&usart_serial_options));
	
	// Higher priority for control signals ?
	usart_enable_interrupt(CONF_UART_CONTROL, UART_IER_RXRDY);
	NVIC_SetPriority(USART0_IRQn, 2);
	NVIC_EnableIRQ(USART0_IRQn);
}

void uart_bridge_send(const uint8_t* data, size_t size) {
	usart_serial_write_packet((Usart *)CONF_UART_CONTROL, data, size);
}

ISR(USART0_Handler)
{	
	static uint8_t value, rcv_buf[128], dec_buf[128];
	static size_t offset = 0;
	static bool transmission_active = false;
	
	usart_serial_read_packet(CONF_UART_CONTROL, &value, 1);
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
			commander_execute(dec_buf[0]);
			offset = 0;
			transmission_active = false;
		}
		else
		{
			rcv_buf[offset++] = value;
			offset %= 128;
		}
	}
	
	uint32_t dummy = usart_get_interrupt_mask(USART0);
}

