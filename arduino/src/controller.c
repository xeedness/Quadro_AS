/*
 * controller.c
 *
 * Created: 27.10.2018 10:46:17
 *  Author: xeedn
 */ 
#include <string.h>
#include "timer.h"
#include "controller.h"
#include "i2c.h"
#include "pid.h"
#include "esc.h"
#include "config.h"
#include "uart_bridge.h"
#include "message_types.h"
#include "state_machine.h"
#include "call_counter.h"

uint8_t request_status[256] = {0};
uint32_t last_alive_ticks;

float thrust = 0;
float target_x = 0;
float target_y = 0;

static void handle_init(uint8_t* payload);
static void handle_alive(void);
static void handle_start(void);
static void handle_stop(void);
static void handle_thrust(float th);
static void handle_controls(float* values);
static void apply_thrust(float thrust);

void setup_controller(void) {
}

bool execute_command(uint8_t cmd, uint8_t* payload)
{
	switch (cmd)
	{
		case MSG_INIT:
		{
			handle_init(payload);
			return true;
		}
		case MSG_ALIVE:
		{
			handle_alive();
			return true;
		}
		case MSG_START:
		{
			handle_start();
			return true;
		}
		case MSG_STOP:
		{
			handle_stop();
			return true;
		}
		case MSG_THRUST:
			handle_thrust(*(float*)(payload));
			return true;
		case MSG_CONTROL:
			handle_controls((float*)(payload));
			return true;
		
		default:
		{
			printf("Received invalid command: %d\n", cmd);
			return false;
		}
	}
}

void handle_init(uint8_t* payload) {
	
	memcpy((char*)&pid_config, (char*)payload, sizeof(pid_config_t));
	memcpy((char*)&log_config, (char*)(payload) + sizeof(pid_config_t), sizeof(log_config_t));	
	memcpy((char*)&esc_config, (char*)(payload) + sizeof(pid_config_t)  + sizeof(log_config_t), sizeof(esc_config_t));
	memcpy((char*)&sensor_config, (char*)(payload) + sizeof(pid_config_t)  + sizeof(log_config_t) + sizeof(esc_config_t), sizeof(sensor_config_t));
	
	// TODO Consider not doing this for better performance
	printf("Received configuration.");
	printf("PID Config: \n");
	printf(" PID-Factor: %.5f\n", pid_config.pid_amplify_factor);
	printf(" P-Factor: %.5f\n", pid_config.pid_angle_p_factor);
	printf(" I-Factor: %.5f\n", pid_config.pid_angle_i_factor);
	printf(" D-Factor: %.5f\n", pid_config.pid_angle_d_factor);
	printf(" P-Factor: %.5f\n", pid_config.pid_rate_p_factor);
	printf(" I-Factor: %.5f\n", pid_config.pid_rate_i_factor);
	printf(" D-Factor: %.5f\n", pid_config.pid_rate_d_factor);
	printf(" Interval-MS: %lu\n", pid_config.update_interval_ms);	
	
	printf("Log Config: \n");
	printf(" Orientation Enabled: %u\n", log_config.orientation_enabled);
	printf(" PID Enabled: %u\n", log_config.pid_enabled);
	printf(" Speed Enabled: %u\n", log_config.speed_enabled);
	printf(" Altitude Enabled: %u\n", log_config.altitude_enabled);
	printf(" Interval-MS: %lu\n", log_config.log_interval_ms);
	
	printf("ESC Config: \n");
	printf(" Landing-Speed: %.5f\n", esc_config.landing_speed);
	printf(" Hover-Speed: %.5f\n", esc_config.hover_speed);
	printf(" Max-Speed: %.5f\n", esc_config.max_speed);
	printf(" Min-Speed: %.5f\n", esc_config.min_speed);
	printf(" Interval-MS: %lu\n", esc_config.update_interval_ms);
	
	printf("Sensor Config: \n");
	printf(" Acceleration Weight: %.5f\n", sensor_config.acceleration_weight);
	printf(" Enabled: %u\n", sensor_config.enabled);
	
	request_status[MSG_INIT] = 1;
}

void  handle_alive(void) {
	last_alive_ticks = current_ticks();
}


void handle_start(void) {
	if(current_state == IDLE || current_state == SHUTDOWN) {
		next_state = RUNNING;	
	}
}
void handle_stop(void) {
	next_state = SHUTDOWN;
}


void handle_thrust(float th) {
	thrust = th;
	apply_thrust(thrust);
}

void handle_controls(float* values) {
	// TODO These values should probably be stored elsewhere
	thrust = values[0];
	target_x = values[1];
	target_y = values[2];
	apply_thrust(thrust);
	set_target(target_x, target_y);
	// TODO Consider not doing this for better performance
	//printf("Controls: T %.5f X %.5f Y %.5f\n", thrust, target_x, target_y);
}

void apply_thrust(float thrust) {
	if(current_state == RUNNING) {
		float maxIncrease = esc_config.max_speed - esc_config.hover_speed;
		float maxDecrease = esc_config.hover_speed - esc_config.min_speed;
		if(thrust > 0) {
			current_base_speed = esc_config.hover_speed + (maxIncrease * thrust);
		} else {
			//th < 0, maxDecrease > 0 -> decrease
			current_base_speed = esc_config.hover_speed + (maxDecrease * thrust);
		}
		//printf("New BaseSpeed: %d\n", BaseSpeed);
	}
}

void relativeMax(float* dst, float* comparison) {
	// Example rr = 0.1, ratio = 2, base = 0.25 : (0.25 - 0.1) * 2 + 0.25 = 0.55
	if(*dst > *comparison) {
		*dst = min(*dst, (current_base_speed - *comparison) * esc_config.max_ratio + current_base_speed);
	}
}

void update_speed(void) {
	speed_called();
	float pid_x = pid_rate_values.x;
	float pid_y = pid_rate_values.y;
		
	//Sum PID values with negated values for right and rear respectively
	speed.front_left_speed = current_base_speed + (-pid_x-pid_y) * pid_config.pid_amplify_factor;
	speed.front_right_speed = current_base_speed + (-pid_x+pid_y) * pid_config.pid_amplify_factor;
	speed.rear_left_speed = current_base_speed + (pid_x-pid_y) * pid_config.pid_amplify_factor;
	speed.rear_right_speed = current_base_speed + (pid_x+pid_y) * pid_config.pid_amplify_factor;
	
	// Absolute MinMax
	speed.front_left_speed = min(speed.front_left_speed, esc_config.max_speed);
	speed.front_right_speed = min(speed.front_right_speed, esc_config.max_speed);
	speed.rear_left_speed = min(speed.rear_left_speed, esc_config.max_speed);
	speed.rear_right_speed = min(speed.rear_right_speed, esc_config.max_speed);
	
	speed.front_left_speed = max(speed.front_left_speed, esc_config.min_speed);
	speed.front_right_speed = max(speed.front_right_speed, esc_config.min_speed);
	speed.rear_left_speed = max(speed.rear_left_speed, esc_config.min_speed);
	speed.rear_right_speed = max(speed.rear_right_speed, esc_config.min_speed);
	
	// Relative Max... 
	relativeMax(&speed.front_left_speed, &speed.rear_right_speed);
	relativeMax(&speed.front_right_speed, &speed.rear_left_speed);
	relativeMax(&speed.rear_left_speed, &speed.front_right_speed);
	relativeMax(&speed.rear_right_speed, &speed.front_left_speed);
	
	
	//printf("Speed: %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n", current_base_speed, pid_x, pid_y, speed.front_left_speed, speed.front_right_speed, speed.rear_left_speed, speed.rear_right_speed);
	
	writeSpeed();
}

uint8_t is_controller_alive(void) {
	return elapsed_time_ms(last_alive_ticks) < 1000;
}

uint8_t request_init(uint8_t is_repeat) {
	if(!is_repeat) {
		request_status[MSG_INIT] = 0;
	}
	return uart_bridge_send_request(MSG_INIT);
}

uint8_t request_init_status(void) {
	return request_status[MSG_INIT];
}