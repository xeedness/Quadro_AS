/*
 * call_counter.c
 *
 * Created: 21.09.2020 23:47:55
 *  Author: xeedn
 */ 

#include <asf.h>
#include <stdio.h>

#include "call_counter.h"
#include "timer.h"


uint32_t sensor_calls, speed_calls, pid_calls;
uint32_t last_output_ticks;

void init_call_counter(void) {
	last_output_ticks = current_ticks();
	sensor_calls = speed_calls = pid_calls = 0;	
}

void sensor_called(void) {
	sensor_calls++;
}

void speed_called(void) {
	speed_calls++;
}
void pid_called(void) {
	pid_calls++;
}

void process_call_counters(void) {
	if(elapsed_time_s(last_output_ticks) > 1.0f) {
		//printf("Calls in the last second:\nSensor: %lu\nSpeed: %lu\nPID: %lu", sensor_calls, speed_calls, pid_calls);
		init_call_counter();
	}
}