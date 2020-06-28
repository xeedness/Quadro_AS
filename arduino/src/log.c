#include <string.h>
#include <stdbool.h>
#include "log.h"
#include "uart_bridge.h"
#include "base85.h"

#define BUF_LENGTH (128)

static bool active = true;

static bool log_write_uart(uint8_t* data, size_t length)
{
	uint8_t buf[BUF_LENGTH + 2] = { 0 };
	
	buf[0] = ASCII_STX;
	memcpy(buf+1, data, length);
	buf[length+1] = ASCII_ETX;
	
	uart_bridge_send(buf, length + 2);
	
	return true;
}

bool log_init()
{
	setup_uart_bridge();
	return true;
}

bool log_bytes(uint8_t* data, size_t length)
{
	uint8_t base85_encoded[BUF_LENGTH] = { 0 };
	int length_encoded;
	
	if((length + ((4 - (length % 4)) % 4)) * 5/4 > BUF_LENGTH)
	{
		return false;
	}
	
	length_encoded = encode_85(base85_encoded, data, length);
	
	return log_write_uart(base85_encoded, length_encoded);
}

bool log_orientation(orientation_t or)
{
	if(!active)
	{
		return false;
	}
	
	size_t or_length = sizeof(orientation_t) + 1;
	uint8_t buf[or_length];
	buf[0] = 42;
	memcpy(buf + 1,	(char*)&or.ax, sizeof(float));
	memcpy(buf + 1 + sizeof(float),	(char*)&or.ay, sizeof(float));
	memcpy(buf + 1 + (2*sizeof(float)), (char*)&or.az, sizeof(float));
	
	return log_bytes(buf, or_length);
}

void log_enable(void)
{
	active = true;
}
void log_disable(void)
{
	active = false;
}
