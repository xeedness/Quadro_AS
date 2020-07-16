/*
 * state_machine.h
 *
 * Created: 13.07.2020 00:10:14
 *  Author: xeedn
 */ 


#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <stdint.h>

enum States {
	AWAIT_INIT = 0,
	IDLE = 1,
	RUNNING = 2,
	LANDING = 3,
	SHUTDOWN = 4,
	END = 5
};



// Number of ticks of the last state update run
extern uint32_t last_state_run_ticks;
// How many ticks has the system been in the current state
extern uint32_t state_ticks;
// How often is the update loop executed
extern uint32_t state_update_interval_ms;

extern uint8_t current_state;
extern uint8_t next_state;

void reset_state(void);
void goto_state(uint32_t newState);

void state_transition(void);

void transition_await_init(void);
void transition_idle(void);
void transition_running(void);
void transition_landing(void);
void transition_shutdown(void);

void timed_state_tasks(void);

void timed_await_init(void);
void timed_idle(void);
void timed_running(void);
void timed_landing(void);
void timed_shutdown(void);

#endif /* STATE_MACHINE_H_ */