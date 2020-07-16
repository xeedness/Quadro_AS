/*
 * log.h
 *
 * Created: 26.06.2020 21:06:44
 *  Author: Philipp
 */ 


#ifndef LOG_H_
#define LOG_H_

#include "sensor.h"
#include "pid.h"
#include "esc.h"

void log_enable(void);
void log_disable(void);

bool log_bytes(const uint8_t* data, size_t length);

bool log_orientation(orientation_t or);
bool log_pid(pid_values_t pid_values);
bool log_thrust(speed_t speed);


#endif /* LOG_H_ */