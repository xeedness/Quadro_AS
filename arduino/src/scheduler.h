/*
 * scheduler.h
 *
 * Created: 24.10.2018 21:07:40
 *  Author: Philipp
 */ 

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <stddef.h>

void scheduler_init(size_t period_ms);
void scheduler_setCB(void (*cb)(void));
void scheduler_clearCB(void);
void scheduler_start(void);
void scheduler_stop(void);

#endif /* SCHEDULER_H_ */