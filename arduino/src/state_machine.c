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
#include "filtering.h"
#include "log.h"

void logging(void);


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
		altimeter_t altimeter_data;
		getFilteredAltitude(&altimeter_data);
		
		orientation_t cur_orientation;
		angular_rate_t cur_angular_rate;
		getFilteredOrientation(&cur_orientation, &cur_angular_rate);
		pid_step(cur_orientation.ax, cur_orientation.ay, cur_angular_rate.wx, cur_angular_rate.wy, cur_angular_rate.wz, altimeter_data.vertical_velocity);
		update_speed();
		last_pid_ticks = current_ticks();
		
		logging();
	}
}

void timed_landing(void) {
	// Currently there is no landing procedure
	next_state = SHUTDOWN;
}

void timed_shutdown(void) {
}


void logging(void) {
	if(log_config.orientation_enabled) {
		orientation_t cur_orientation;
		angular_rate_t cur_angular_velocity;
		getFilteredOrientation(&cur_orientation, &cur_angular_velocity);
			

		cur_orientation.ax /= DEG_TO_RAD_FACTOR;
		cur_orientation.ay /= DEG_TO_RAD_FACTOR;
		
		float tx,ty,tz;
		// Log target instead of orientation
		get_target(&tx, &ty);
		tz = 0;
		log_target(tx, ty, tz);
		log_orientation(cur_orientation);
		log_angular_velocity(cur_angular_velocity);
		
	}
		
	if(log_config.pid_enabled) {
		//printf("PID Logging: %.2f %.2f\n", pid_rate_values.x, pid_rate_values.y);
		log_pid(pid_rate_values);
	}
		
	if(log_config.speed_enabled) {
		//printf("Speed Logging: %d %d %d %d\n", speed.front_left_speed, speed.front_right_speed, speed.rear_left_speed, speed.rear_right_speed);
		log_thrust(speed);
	}
		
	if(log_config.altitude_enabled) {
		altimeter_t altimeter_data;
		getFilteredAltitude(&altimeter_data);
		log_altimeter_data(altimeter_data);
	}
	
	#define STATUS_INTERVAL_MS 1000
	//#define ENABLE_UART_STATUS
	#ifdef ENABLE_UART_STATUS
	if(elapsed_time_ms(last_uart_ticks) > STATUS_INTERVAL_MS) {
		switch(current_state) {
			case(AWAIT_INIT):
			printf("Await Init\n");
			break;
			case(IDLE):
			printf("Idle\n");
			break;
			case(RUNNING):
			printf("Running\n");
			break;
			case(LANDING):
			printf("Landing\n");
			break;
			case(SHUTDOWN):
			printf("Shutdown\n");
			break;
			default:
			printf("No State\n");
		}
		printf("Thrust: %.2f\n", get_target_vertical_velocity());
		float target_x, target_y;
		get_target(&target_x, &target_y);
		printf("Target: %.2f %.2f\n", target_x, target_y);
		last_uart_ticks = current_ticks();
	}
	#endif
}