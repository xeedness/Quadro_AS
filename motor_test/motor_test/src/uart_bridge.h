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
#include <stdbool.h>

#define ASCII_STX ((char) 0x2)
#define ASCII_ETX ((char) 0x3)

void uart_bridge_init(void);
void uart_bridge_send_raw(const uint8_t* data, size_t size);
bool uart_bridge_send(const uint8_t* data, size_t size);
bool uart_bridge_send_request(const uint8_t type);


#endif /* UART_BRIDGE_H_ */