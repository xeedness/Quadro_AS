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
#include "kalman.h"

void log_enable(void);
void log_disable(void);

bool log_bytes(const uint8_t* data, size_t length);

bool log_orientation(orientation_t or);
bool log_angular_velocity(angular_rate_t av);
bool log_pid(pid_values_t pid_values);
bool log_thrust(speed_t speed);
bool log_altimeter_data(altimeter_t altimeter_data);


#endif /* LOG_H_ */