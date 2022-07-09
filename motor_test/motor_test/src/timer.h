/*
 * timer.h
 *
 * Created: 13.07.2020 20:11:15
 *  Author: xeedn
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

void timer_init(void);
uint32_t current_ticks(void);
void increment_state_ticks(void);
uint32_t elapsed_time_us(uint32_t past);
uint32_t elapsed_time_ms(uint32_t past);
float elapsed_time_s(uint32_t past);



#endif /* TIMER_H_ */