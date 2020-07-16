#include <string.h>
#include <stdbool.h>
#include "log.h"
#include "uart_bridge.h"
#include "base85.h"
#include "message_types.h"

bool log_bytes(const uint8_t* data, size_t length) {
	return uart_bridge_send(data, length);
}

bool log_orientation(orientation_t or)
{	
	size_t or_length = sizeof(orientation_t) + 1;
	uint8_t buf[or_length];
	buf[0] = MSG_LOG_ORIENTATION;
	memcpy(buf + 1,	(char*)&or.ax, sizeof(orientation_t));	
	return log_bytes(buf, or_length);
}

bool log_pid(pid_values_t pid_val)
{
	size_t pid_length = sizeof(pid_values_t) + 1;
	uint8_t buf[pid_length];
	buf[0] = MSG_LOG_PID;
	memcpy(buf + 1,	(char*)&pid_val, sizeof(pid_values_t));
	
	return log_bytes(buf, pid_length);
}

bool log_thrust(speed_t spd) {	
	size_t thrust_length = sizeof(speed_t) + 1;
	uint8_t buf[thrust_length];
	buf[0] = MSG_LOG_THRUST;
	memcpy(buf + 1, (char*)&spd, sizeof(speed_t));	
	return log_bytes(buf, thrust_length);
}
