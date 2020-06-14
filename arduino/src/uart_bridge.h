/*
 * uart_bridge.h
 *
 * Created: 14.06.2020 20:26:21
 *  Author: xeedn
 */ 

#ifndef UART_BRIDGE_H_
#define UART_BRIDGE_H_

#include <stdint.h>
#include <stdlib.h>

void setup_uart_bridge(void);
void uart_bridge_send(const uint8_t* data, size_t size);



#endif /* UART_BRIDGE_H_ */