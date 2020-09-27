/*
 * call_counter.h
 *
 * Created: 21.09.2020 23:46:24
 *  Author: xeedn
 */ 


#ifndef CALL_COUNTER_H_
#define CALL_COUNTER_H_

void init_call_counter(void);
void process_call_counters(void);

void sensor_called(void);
void speed_called(void);
void pid_called(void);


#endif /* CALL_COUNTER_H_ */