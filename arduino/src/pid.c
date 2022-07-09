/*
 * pid.c
 *
 * Created: 14.10.2018 18:17:09
 *  Author: xeedn
 */ 

#include "pid.h"
#include "timer.h"
#include "config.h"
#include "call_counter.h"

pid_values_t pid_angle_values = {0, 0};
pid_values_t pid_rate_values = {0, 0};

float last_angle_error_x;
float last_angle_error_y;
float last_rate_error_x;
float last_rate_error_y;
float i_value_angle_x;
float i_value_angle_y;
float i_value_rate_x;
float i_value_rate_y;
uint32_t last_pid_tick;
float target_x, target_y, target_z;

uint8_t mode = MODE_ROTATION;

typedef struct pid_state {
	float p, i, d;
	float last_error;
	float i_value;
	float value;
} pid_state_t;

float vertical_velocity_target = 0;

pid_state_t orientation_x_pid_state;
pid_state_t orientation_y_pid_state;
pid_state_t angular_rate_x_pid_state;
pid_state_t angular_rate_y_pid_state;
pid_state_t angular_rate_z_pid_state;
pid_state_t vertical_velocity_pid_state;

void angle_pid_step(pid_state_t* state, float error, float elapsed_seconds);
void generic_pid_step(pid_state_t* state, float error, float elapsed_seconds);

 void pid_init(void) {
	 //pid_values = {0, 0};
	 target_x = target_y = target_z = 0;	 
	 last_pid_tick = current_ticks();
	 
	 vertical_velocity_pid_state.last_error = 0;
	 vertical_velocity_pid_state.i_value = 0;
	 vertical_velocity_pid_state.value = 0;
	 
	 orientation_x_pid_state.last_error = 0;
	 orientation_x_pid_state.i_value = 0;
	 orientation_x_pid_state.value = 0;
	 
	 orientation_y_pid_state.last_error = 0;
	 orientation_y_pid_state.i_value = 0;
	 orientation_y_pid_state.value = 0;
	 
	 angular_rate_x_pid_state.last_error = 0;
	 angular_rate_x_pid_state.i_value = 0;
	 angular_rate_x_pid_state.value = 0;
	 
	 angular_rate_y_pid_state.last_error = 0;
	 angular_rate_y_pid_state.i_value = 0;
	 angular_rate_y_pid_state.value = 0;
	 
	 angular_rate_z_pid_state.last_error = 0;
	 angular_rate_z_pid_state.i_value = 0;
	 angular_rate_z_pid_state.value = 0;
 }
 
 static void pid_angle_step(float x_angle, float y_angle, float elapsed_seconds) {
	 if(mode == MODE_ORIENTATION || (mode == MODE_HYBRID && abs(target_x) < 0.03f)) {
		 orientation_x_pid_state.p = pid_config.pid_angle_p_factor;
		 orientation_x_pid_state.i = pid_config.pid_angle_i_factor;
		 orientation_x_pid_state.d = pid_config.pid_angle_d_factor;
		 angle_pid_step(&orientation_x_pid_state, target_x-x_angle, elapsed_seconds);
		 pid_angle_values.x = orientation_x_pid_state.value;
	 }
	 
	 if(mode == MODE_ORIENTATION || (mode == MODE_HYBRID && abs(target_y) < 0.03f)) {
		 orientation_y_pid_state.p = pid_config.pid_angle_p_factor;
		 orientation_y_pid_state.i = pid_config.pid_angle_i_factor;
		 orientation_y_pid_state.d = pid_config.pid_angle_d_factor;
		 angle_pid_step(&orientation_y_pid_state, target_y-y_angle, elapsed_seconds);
		 pid_angle_values.y = orientation_y_pid_state.value;
	 }
 }
 
 static void pid_rate_step(float x_rate, float y_rate, float z_rate, float elapsed_seconds) {
	 angular_rate_x_pid_state.p = pid_config.pid_rate_p_factor;
	 angular_rate_x_pid_state.i = pid_config.pid_rate_i_factor;
	 angular_rate_x_pid_state.d = pid_config.pid_rate_d_factor;
	 
	 angular_rate_y_pid_state.p = pid_config.pid_rate_p_factor;
	 angular_rate_y_pid_state.i = pid_config.pid_rate_i_factor;
	 angular_rate_y_pid_state.d = pid_config.pid_rate_d_factor;
	 
	 angular_rate_z_pid_state.p = pid_config.pid_rate_p_factor;
	 angular_rate_z_pid_state.i = pid_config.pid_rate_i_factor;
	 angular_rate_z_pid_state.d = pid_config.pid_rate_d_factor;
	 	 
	 if(mode == MODE_ORIENTATION || (mode == MODE_HYBRID && abs(target_x) < 0.03f)) {
		 generic_pid_step(&angular_rate_x_pid_state, pid_angle_values.x-x_rate, elapsed_seconds);	 
	 } else {
		 generic_pid_step(&angular_rate_x_pid_state, target_x-x_rate, elapsed_seconds);
	 }
	 
	 if(mode == MODE_ORIENTATION || (mode == MODE_HYBRID && abs(target_y) < 0.03f)) {
		 generic_pid_step(&angular_rate_y_pid_state, pid_angle_values.y-y_rate, elapsed_seconds);
	 } else {
		 generic_pid_step(&angular_rate_y_pid_state, target_y-y_rate, elapsed_seconds);
	 }
	 
	 
	 
	 generic_pid_step(&angular_rate_z_pid_state, target_z-z_rate, elapsed_seconds);
	 

	 pid_rate_values.x = angular_rate_x_pid_state.value;
	 pid_rate_values.y = angular_rate_y_pid_state.value;
	 pid_rate_values.z = angular_rate_z_pid_state.value;
 }
 
static void pid_vertical_velocity_step(float vertical_velocity, float elapsed_seconds) {
	vertical_velocity_pid_state.p = pid_config.pid_vertical_velocity_p_factor;
	vertical_velocity_pid_state.i = pid_config.pid_vertical_velocity_i_factor;
	vertical_velocity_pid_state.d = pid_config.pid_vertical_velocity_d_factor;
	generic_pid_step(&vertical_velocity_pid_state, vertical_velocity_target-vertical_velocity, elapsed_seconds);
}
 
void generic_pid_step(pid_state_t* state, float error, float elapsed_seconds) {
	 float p_value, d_value;
	 
	 p_value = error;
	 state->i_value += elapsed_seconds * error;
	 d_value = (error-state->last_error)/elapsed_seconds;
	 
	 state->last_error = error;
	 
	 state->value = state->p*p_value + state->i*state->i_value + state->d*d_value;
 }
 
 void angle_pid_step(pid_state_t* state, float error, float elapsed_seconds) {
	 float p_value, d_value;
	 
	 p_value = error;
	 state->i_value += elapsed_seconds * error;

#define ANGLE_ERROR_BOUNDARY 1.3f*3.14f
	
	// Take into account wrap around of angles
	 if((abs(error) > ANGLE_ERROR_BOUNDARY || abs(state->last_error) > ANGLE_ERROR_BOUNDARY)
	 && ((error < 0 && state->last_error > 0) || (error > 0 && state->last_error < 0))) {
		 if(error < 0) {
			d_value = ((error+6.283f)-state->last_error)/elapsed_seconds;
		 } else {
			d_value = ((error-6.283f)-state->last_error)/elapsed_seconds; 
		 }
	 } else {
		 // The usual formular
		d_value = (error-state->last_error)/elapsed_seconds;
	 }
	 
	 state->last_error = error;
	 
	 state->value = state->p*p_value + state->i*state->i_value + state->d*d_value;
 }
 
 void pid_step(float x_angle, float y_angle, float x_rate, float y_rate, float z_rate, float vertical_velocity) {
	 float elapsed_seconds = elapsed_time_s(last_pid_tick);
	 last_pid_tick = current_ticks();
	 if(elapsed_seconds > 1.0f) {
		 elapsed_seconds = 1.0f;
	 }
	 if(elapsed_seconds > 0) {
		 pid_called();
		 pid_angle_step(x_angle, y_angle, elapsed_seconds);
		 pid_rate_step(x_rate, y_rate, z_rate, elapsed_seconds);
		 pid_vertical_velocity_step(vertical_velocity, elapsed_seconds);
	 }
 }
 
 void set_target(float x, float y) {
	 target_x = x;
	 target_y = y;
 }
 
 void get_target(float* x, float* y) {
	 *x = target_x;
	 *y = target_y;
 }
 
 void set_target_vertical_velocity(float vertical_velocity) {
	 vertical_velocity_target = vertical_velocity;
 }
 
 float get_target_vertical_velocity(void) {
	 return vertical_velocity_target;
 }
 
 void set_mode(uint8_t in_mode) {
	 mode = in_mode;
 }
 
 uint8_t get_mode(void) {
	 return mode;
 }
 
