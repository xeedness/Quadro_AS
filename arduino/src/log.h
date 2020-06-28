/*
 * log.h
 *
 * Created: 26.06.2020 21:06:44
 *  Author: Philipp
 */ 


#ifndef LOG_H_
#define LOG_H_

#include "sensor.h"

bool log_init(void);

void log_enable(void);
void log_disable(void);

bool log_bytes(uint8_t* data, size_t length);

bool log_orientation(orientation_t or);


#endif /* LOG_H_ */