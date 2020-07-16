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

uint8_t request_status[256] = {0};
uint32_t last_alive_ticks;

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
		
		default:
		{
			return false;
		}
	}
}

void handle_init(uint8_t* payload) {
	
	memcpy((char*)&pid_config, (char*)payload, sizeof(pid_config_t));
	memcpy((char*)&log_config, (char*)(payload) + sizeof(pid_config_t), sizeof(log_config_t));	
	memcpy((char*)&esc_config, (char*)(payload) + sizeof(pid_config_t) +  + sizeof(log_config_t), sizeof(esc_config_t));
	
	
	printf("Received configuration.");
	printf("PID Config: \n");
	printf(" PID-Factor: %.5f\n", pid_config.pid_factor);
	printf(" P-Factor: %.5f\n", pid_config.pid_p_factor);
	printf(" I-Factor: %.5f\n", pid_config.pid_i_factor);
	printf(" D-Factor: %.5f\n", pid_config.pid_d_factor);
	printf(" Interval-MS: %lu\n", pid_config.update_interval_ms);	
	
	printf("Log Config: \n");
	printf(" Orientation Enabled: %u\n", log_config.orientation_enabled);
	printf(" PID Enabled: %u\n", log_config.pid_enabled);
	printf(" Speed Enabled: %u\n", log_config.speed_enabled);
	printf(" Interval-MS: %lu\n", log_config.log_interval_ms);
	
	printf("ESC Config: \n");
	printf(" Landing-Speed: %u\n", esc_config.landing_speed);
	printf(" Hover-Speed: %u\n", esc_config.hover_speed);
	printf(" Max-Speed: %u\n", esc_config.max_speed);
	printf(" Min-Speed: %u\n", esc_config.min_speed);
	printf(" Interval-MS: %lu\n", esc_config.update_interval_ms);
	
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
	if(current_state == RUNNING) {
		int maxIncrease = esc_config.max_speed - esc_config.hover_speed;
		int maxDecrease = esc_config.hover_speed - esc_config.min_speed;
		if(th > 0) {
			current_base_speed = esc_config.hover_speed + (maxIncrease * th);
		} else {
			//th < 0, maxDecrease > 0 -> decrease
			current_base_speed = esc_config.hover_speed + (maxDecrease * th);
		}
		//printf("New BaseSpeed: %d\n", BaseSpeed);
	}
}

void update_speed(void) {
	float pid_x = pid_values.x;
	float pid_y = pid_values.y;
	
	//Sum PID values with negated values for right and rear respectively
	speed.front_left_speed = current_base_speed + (-pid_x/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW)) + (-pid_y/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW));
	speed.front_right_speed = current_base_speed + (-pid_x/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW)) + (pid_y/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW));
	speed.rear_left_speed = current_base_speed + (pid_x/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW)) + (-pid_y/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW));
	speed.rear_right_speed = current_base_speed + (pid_x/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW)) + (pid_y/100.0f * pid_config.pid_factor * (ESC_HIGH-ESC_LOW));
	
	speed.front_left_speed = min(speed.front_left_speed, esc_config.max_speed);
	speed.front_right_speed = min(speed.front_right_speed, esc_config.max_speed);
	speed.rear_left_speed = min(speed.rear_left_speed, esc_config.max_speed);
	speed.rear_right_speed = min(speed.rear_right_speed, esc_config.max_speed);
	
	speed.front_left_speed = max(speed.front_left_speed, esc_config.min_speed);
	speed.front_right_speed = max(speed.front_right_speed, esc_config.min_speed);
	speed.rear_left_speed = max(speed.rear_left_speed, esc_config.min_speed);
	speed.rear_right_speed = max(speed.rear_right_speed, esc_config.min_speed);
	
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