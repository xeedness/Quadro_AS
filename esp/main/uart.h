#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdlib.h>

#define BUF_SIZE (1024)

void uart_init(void);
int uart_send(uint8_t* data, size_t data_size);
int uart_recv(uint8_t* data, size_t data_size);

#endif