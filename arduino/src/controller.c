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

static void handle_init(uint8_t* payload);
static void handle_alive(void);
static void handle_start(void);
static void handle_stop(void);
static void handle_thrust(float* th);
static void handle_controls(float* values);
static void apply_thrust(float thrust);

uint8_t input_config;

void update_speed_testbench(void);

void controller_init(uint8_t i_input_config) {
	input_config = i_input_config;
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
			handle_thrust((float*)(payload));
			return true;
		case MSG_CONTROL:
			handle_controls((float*)(payload));
			return true;
		
		default:
		{
			//printf("Received invalid command: %d\n", cmd);
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
	printf(" P-Factor: %.5f\n", pid_config.pid_vertical_velocity_p_factor);
	printf(" I-Factor: %.5f\n", pid_config.pid_vertical_velocity_i_factor);
	printf(" D-Factor: %.5f\n", pid_config.pid_vertical_velocity_d_factor);
	printf(" Interval-MS: %lu\n", pid_config.update_interval_ms);	
	
	printf("Log Config: \n");
	printf(" Orientation Enabled: %u\n", log_config.orientation_enabled);
	printf(" PID Enabled: %u\n", log_config.pid_enabled);
	printf(" Speed Enabled: %u\n", log_config.speed_enabled);
	printf(" Altitude Enabled: %u\n", log_config.altitude_enabled);
	printf(" Interval-MS: %lu\n", log_config.log_interval_ms);
	
	printf("ESC Config: \n");
	printf(" Min-Speed: %.5f\n", esc_config.min_speed);
	printf(" Max-Speed: %.5f\n", esc_config.max_speed);
	printf(" PID Amplifier: %.5f\n", esc_config.vertical_velocity_pid_amplifier);
	printf(" Velocity-Limit: %.5f\n", esc_config.vertical_velocity_limit);	
	printf(" Interval-MS: %lu\n", esc_config.update_interval_ms);
	
	printf("Sensor Config: \n");
	printf(" Acceleration Weight: %.5f\n", sensor_config.acceleration_weight);
	printf(" Measurement Error Angle Weight: %.5f\n", sensor_config.measurement_error_angle);
	printf(" Measurement Error Angular Velocity: %.5f\n", sensor_config.measurement_error_angular_velocity);
	printf(" Estimate Error Angle Weight: %.5f\n", sensor_config.estimate_error_angle);
	printf(" Estimate Error Angular Velocity: %.5f\n", sensor_config.estimate_error_angular_velocity);
	printf(" Altitude Gain: %.5f\n", sensor_config.altitude_gain);
	printf(" Speed Gain: %.5f\n", sensor_config.speed_gain);
	printf(" Acceleration Weight: %.5f\n", sensor_config.acceleration_weight);
	printf(" Use-Kalman-Orientation: %u\n", sensor_config.use_kalman_orientation);
	printf(" Enabled: %u\n", sensor_config.enabled);
	
	request_status[MSG_INIT] = 1;
}

void  handle_alive(void) {
	last_alive_ticks = current_ticks();
}


void handle_start(void) {
	if(!(input_config & ENABLE_ESP_POWER)) {
		return;
	}
	if(current_state == IDLE || current_state == SHUTDOWN) {
		next_state = RUNNING;	
	}
}
void handle_stop(void) {
	// Enable for safety reasons
	next_state = SHUTDOWN;
}


void handle_thrust(float* th) {
	if(!(input_config & ENABLE_ESP_THROTTLE)) {
		return;
	}
	apply_thrust(*th);
	last_alive_ticks = current_ticks();
}

void handle_controls(float* values) {
	float thrust = values[0];
	float target_x = values[1];
	float target_y = values[2];
	if(input_config & ENABLE_ESP_THROTTLE) {
		apply_thrust(thrust);
	}
	if(input_config & ENABLE_ESP_ROTATION) {
		set_target(target_x, target_y);
	}
	last_alive_ticks = current_ticks();
	//printf("Controls: T %.5f X %.5f Y %.5f\n", thrust, target_x, target_y);
}

void apply_thrust(float thrust) {
	if(current_state == RUNNING) {
		float max_vertical_velocity = 2;
		set_target_vertical_velocity(max_vertical_velocity * thrust);
	}
}

void relativeMax(float* dst, float* comparison) {
	// Example rr = 0.1, ratio = 2, base = 0.25 : (0.25 - 0.1) * 2 + 0.25 = 0.55
	/*float max_ratio = 2;
	if(*dst > *comparison) {
		*dst = min(*dst, (current_base_speed - *comparison) * max_ratio + current_base_speed);
	}*/
}

void update_speed_testbench(void) {
	float pid_y = pid_rate_values.y;
	
	// TMP FOR TESTBENCH
	float pid_speed = 0.3f;
	
	// Trim base speed values to allow for orientation correction. Otherwise the motors are reaching their limits on one side
	pid_speed = min(esc_config.max_speed-esc_config.vertical_velocity_limit, pid_speed);
	pid_speed = max(esc_config.min_speed+esc_config.vertical_velocity_limit, pid_speed);
	
	// TMP FOR TESTBENCH
	speed.front_left_speed = pid_speed + ((pid_y) * pid_config.pid_amplify_factor);
	speed.front_right_speed = pid_speed + ((-pid_y) * pid_config.pid_amplify_factor);
	speed.rear_left_speed = pid_speed + ((pid_y) * pid_config.pid_amplify_factor);
	speed.rear_right_speed = pid_speed + ((-pid_y) * pid_config.pid_amplify_factor);
	
	// Absolute MinMax
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

void update_speed(void) {
	speed_called();
	update_speed_testbench();
	return;
	float pid_x = pid_rate_values.x;
	float pid_y = pid_rate_values.y;
	float pid_z = pid_rate_values.z;
	
	float pid_speed = pid_vertical_velocity_value * esc_config.vertical_velocity_pid_amplifier;
	
	// Trim base speed values to allow for orientation correction. Otherwise the motors are reaching their limits on one side
	pid_speed = min(esc_config.max_speed-esc_config.vertical_velocity_limit, pid_speed);
	pid_speed = max(esc_config.min_speed+esc_config.vertical_velocity_limit, pid_speed);
	
	//Sum PID values with negated values for right and rear respectively
	speed.front_left_speed = pid_speed + ((pid_x+pid_y) * pid_config.pid_amplify_factor);
	speed.front_right_speed = pid_speed + ((pid_x-pid_y) * pid_config.pid_amplify_factor);
	speed.rear_left_speed = pid_speed + ((-pid_x+pid_y) * pid_config.pid_amplify_factor);
	speed.rear_right_speed = pid_speed + ((-pid_x-pid_y) * pid_config.pid_amplify_factor);
	
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
	//relativeMax(&speed.front_left_speed, &speed.rear_right_speed);
	//relativeMax(&speed.front_right_speed, &speed.rear_left_speed);
	//relativeMax(&speed.rear_left_speed, &speed.front_right_speed);
	//relativeMax(&speed.rear_right_speed, &speed.front_left_speed);
	
	//printf("Speed: %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n", current_base_speed, pid_x, pid_y, speed.front_left_speed, speed.front_right_speed, speed.rear_left_speed, speed.rear_right_speed);
	
	/*if(speed.front_left_speed < 0.2) {
		speed.front_left_speed = 0;
	}
	if(speed.front_right_speed < 0.2) {
		speed.front_right_speed = 0;
	}

	if(speed.rear_left_speed < 0.2) {
		speed.rear_left_speed = 0;
	}

	if(speed.rear_right_speed < 0.2) {
		speed.rear_right_speed = 0;
	}*/
	
	writeSpeed();
}

uint8_t is_controller_alive(void) {
	return elapsed_time_ms(last_alive_ticks) < 2000;
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

//static uint32_t interpreted = 0;
void interpret_channels(uint16_t* data, uint8_t channel_count) {	
	/*if(interpreted > 100) {
		printf("Channel Values: %d %d %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
		interpreted = 0;
	}
	interpreted++;*/
	
	// Flightmode
	if(channel_count >= 9 && input_config & ENABLE_IBUS_FLIGHTMODE) {
		// Note: Usual 3 state switch values are 1000, 1500 and 2000.
		if(data[8] <= 1250) {
			set_mode(MODE_ROTATION);
		} else if(data[8] <= 1750) {
			set_mode(MODE_HYBRID);
		} else {
			set_mode(MODE_ORIENTATION);
		}
	}
	
	
	// Thrust
	if(channel_count >= 4 && input_config & ENABLE_IBUS_THROTTLE && current_state == RUNNING) {
		float thrust = ((float)(data[2])-1000.0f)/1000.0f;
		// Ignore 3 (z rotation) for now
		apply_thrust(thrust);
	}
	
	// Rotation modifier
	float x_rot_modifier = 3.14f/12.0f;
	float y_rot_modifier = 3.14f/12.0f;
	if(channel_count >= 6) {
		// TODO Find proper base values here
		x_rot_modifier = 3.14f * ((float)(data[4])-1000.0f)/1000.0f;
		y_rot_modifier = 3.14f * ((float)(data[5])-1000.0f)/1000.0f;
	}
	
	// Rotation
	if(channel_count >= 2 && input_config & ENABLE_IBUS_ROTATION && current_state == RUNNING) {
		if(get_mode() == MODE_ORIENTATION) {
			// Explicitly do not use modifiers just now. I consider using the modifiers and switching between modes a little unstable
			float target_y = 3.14f/3.0f * ((float)(data[0])-1500.0f)/500.0f;
			float target_x = 3.14f/3.0f * ((float)(data[1])-1500.0f)/500.0f;
			set_target(target_x, target_y);
			
		} else {
			float target_y = x_rot_modifier * ((float)(data[0])-1500.0f)/500.0f;
			float target_x = y_rot_modifier * ((float)(data[1])-1500.0f)/500.0f;
			set_target(target_x, target_y);	
		}
	}
	
	//TODO Z Rotation
	
	// On/Off
	if(channel_count >= 7 && input_config & ENABLE_IBUS_POWER) {
		// Note: Usual two state switch values are 1000 and 2000.
		if(data[6] <= 1500) {
			if(current_state > IDLE) {
				next_state = IDLE;
			}
		} else {
			if(current_state == IDLE) {
				next_state = RUNNING;
			}
		}
	}
	last_alive_ticks = current_ticks();
}