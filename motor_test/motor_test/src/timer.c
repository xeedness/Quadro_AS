/*
 * timer.c
 *
 * Created: 13.07.2020 20:11:29
 *  Author: xeedn
 */ 

#include "timer.h"


volatile uint32_t myticks; //1 tick == 10 us

void timer_init(void) {
	myticks = 0;
}

uint32_t current_ticks(void) {
	return myticks;
}

void increment_state_ticks(void) {
	myticks++;
}

uint32_t elapsed_time_us(uint32_t past) {
	return (myticks-past)*1000;
}

uint32_t elapsed_time_ms(uint32_t past) {
	return (myticks-past);
}

float elapsed_time_s(uint32_t past) {
	return (float)(myticks-past)/1000.0f;
}