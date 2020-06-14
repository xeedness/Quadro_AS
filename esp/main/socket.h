#ifndef SOCKET_H
#define SOCKET_H

#include <stdint.h>
#include <stdlib.h>

int connect_send_socket(void);
int connect_recv_socket(void);
void kill_send_socket(void);
void kill_recv_socket(void);
int tcp_socket_send(uint8_t* data, int length);
int tcp_socket_recv(uint8_t* data, int length);

#endif