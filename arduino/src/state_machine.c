/*
 * state_machine.c
 *
 * Created: 13.07.2020 00:10:34
 *  Author: xeedn
 */ 

#include "state_machine.h"

#include <asf.h>
#include "timer.h"
#include "esc.h"
#include "config.h"
#include "controller.h"
#include "sensor.h"
#include "pid.h"
#include "status_display.h"
#include "kalman.h"


uint32_t last_pid_ticks = 0;
uint32_t last_speed_ticks = 0;

static const uint32_t state_update_intervals_ms[END] = {
	2000, // AWAIT_INIT
	1000, // IDLE
	1, // RUNNING
	1, // LANDING
	0 // SHUTDOWN
};

// Number of ticks of the last state update run
 uint32_t last_state_run_ticks = 0;
// How many ticks has the system been in the current state
 uint32_t state_ticks;
// How often is the update loop executed
 uint32_t state_update_interval_ms;

 uint8_t current_state;
 uint8_t next_state;

void reset_state(void) {
	last_pid_ticks = current_ticks();
	last_speed_ticks = current_ticks();
	current_state = -1;
	next_state = AWAIT_INIT;
}

void goto_state(uint32_t newState) {
	current_state = newState;
	state_ticks = current_ticks();
	state_update_interval_ms = state_update_intervals_ms[current_state];
}

void state_transition(void) {
	if(current_state != next_state) {
		//Process next state
		switch(next_state) {
			case(AWAIT_INIT):
				transition_await_init();
				break;
			case(IDLE):
				transition_idle();
				break;
			case(RUNNING):
				transition_running();
				break;
			case(LANDING):
				transition_landing();
				break;
			case(SHUTDOWN):
				transition_shutdown();
				break;
		}
		goto_state(next_state);
	}
	
}

void transition_await_init(void) {
	set_init_led(0);
	request_init(0);
	minThrottle();
}

void transition_idle(void) {
	set_init_led(1);
	minThrottle();
}

void transition_running(void) {
	printf("Running...");
	pid_init();
	current_base_speed = esc_config.hover_speed;
}

void transition_landing(void) {
	printf("Landing...");
	// Currently there is no landing procedure
}

void transition_shutdown(void) {
	printf("Shutting down...");
	minThrottle();
}

void timed_state_tasks(void) {
	if(state_update_interval_ms > 0 && elapsed_time_ms(last_state_run_ticks) > state_update_interval_ms) {
		switch(current_state) {
			case(AWAIT_INIT):
				timed_await_init();
				break;
			case(IDLE):
				timed_idle();
				break;
			case(RUNNING):
				timed_running();
				break;
			case(LANDING):
				timed_landing();
				break;
			case(SHUTDOWN):
				timed_shutdown();
				break;
			default:
				printf("No State\n");
		}
		last_state_run_ticks = current_ticks();
	}
}

void timed_await_init(void) {
	printf("Requesting initialization.\n");
	if(request_init_status()) {
		next_state = IDLE;
	} else {
		request_init(1);
	}
}

void timed_idle(void) {
	printf("Idle\n");
}

void timed_running(void) {
	if(!is_controller_alive()) {
		printf("Controller is dead.\n");
		next_state = SHUTDOWN;
	}
	if(!is_sensor_alive()) {
		printf("Sensor is dead.\n");
		next_state = SHUTDOWN;
	}
	if(elapsed_time_ms(last_pid_ticks) > pid_config.update_interval_ms) {
		//printf("Updating angles: %.2f %.2f\n", current_orientation.ax, current_orientation.ay);
		orientation_t cur_orientation;
		angular_rate_t cur_angular_rate;
		getKalmanOrientationEstimate(&cur_orientation, &cur_angular_rate);
		pid_step(cur_orientation.ax, cur_orientation.ay, cur_angular_rate.wx, cur_angular_rate.wy);
		last_pid_ticks = current_ticks();
	}
	if(elapsed_time_ms(last_speed_ticks) > esc_config.update_interval_ms) {
		update_speed();
		last_speed_ticks = current_ticks();
	}
}

void timed_landing(void) {
	// Currently there is no landing procedure
	next_state = SHUTDOWN;
}

void timed_shutdown(void) {
}