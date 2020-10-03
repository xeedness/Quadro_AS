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
float pid_vertical_velocity_value = 0.1f;

float last_angle_error_x;
float last_angle_error_y;
float last_rate_error_x;
float last_rate_error_y;
float i_value_angle_x;
float i_value_angle_y;
float i_value_rate_x;
float i_value_rate_y;
uint32_t last_pid_tick;
float target_angle_x, target_angle_y;

float last_vertical_velocity_error;
float i_value_vertical_velocity;
float target_vertical_velocity;

 void pid_init(void) {
	 //pid_values = {0, 0};
	 target_angle_x = target_angle_y = 0;
	 i_value_angle_x = i_value_angle_y = 0;
	 i_value_rate_x = i_value_rate_y = 0;
	 last_angle_error_x = last_angle_error_y = 0;
	 last_rate_error_x = last_rate_error_y = 0;
	 last_vertical_velocity_error = 0;
	 i_value_vertical_velocity = 0;
	 target_vertical_velocity = 0;
	 last_pid_tick = current_ticks();
 }
 
 static void pid_angle_step(float x_angle, float y_angle, float elapsed_seconds) {
	 float error_x, p_value_x, d_value_x;
	 float error_y, p_value_y, d_value_y;
	 
	 //Calc error
	 error_x = target_angle_x-x_angle;
	 error_y = target_angle_y-y_angle;
	 
	 // P Value is the error directly
	 p_value_x = error_x;
	 p_value_y = error_y;
	 
	 // I Value is the discrete integral (sum) of the product of error and time
	 i_value_angle_x += elapsed_seconds * error_x;
	 i_value_angle_y += elapsed_seconds * error_y;
	 
	 // D Value is the change of error
	 d_value_x = (error_x-last_angle_error_x)/elapsed_seconds;
	 d_value_y = (error_y-last_angle_error_y)/elapsed_seconds;
	 
	 // Generate pid value by weighted sum
	 pid_angle_values.x = pid_config.pid_angle_p_factor*p_value_x+pid_config.pid_angle_i_factor*i_value_angle_x+pid_config.pid_angle_d_factor*d_value_x;
	 pid_angle_values.y = pid_config.pid_angle_p_factor*p_value_y+pid_config.pid_angle_i_factor*i_value_angle_y+pid_config.pid_angle_d_factor*d_value_y;
	 
	 // Remember the last error to calc next values
	 last_angle_error_x = error_x;
	 last_angle_error_y = error_y;
 }
 
 static void pid_rate_step(float x_rate, float y_rate, float elapsed_seconds) {
	 float error_x, p_value_x, d_value_x;
	 float error_y, p_value_y, d_value_y;
	 
	 //Calc error // Sign swap, because of the way motors a controlled
	 error_x = x_rate-pid_angle_values.x;
	 error_y = y_rate-pid_angle_values.y;

	 // P Value is the error directly
	 p_value_x = error_x;
	 p_value_y = error_y;
	 
	 // I Value is the discrete integral (sum) of the product of error and time
	 i_value_rate_x += elapsed_seconds * error_x;
	 i_value_rate_y += elapsed_seconds * error_y;
	 
	 // D Value is the change of error
	 d_value_x = (error_x-last_rate_error_x)/elapsed_seconds;
	 d_value_y = (error_y-last_rate_error_y)/elapsed_seconds;
	 
	 // Generate pid value by weighted sum
	 pid_rate_values.x = pid_config.pid_rate_p_factor*p_value_x+pid_config.pid_rate_i_factor*i_value_rate_x+pid_config.pid_rate_d_factor*d_value_x;
	 pid_rate_values.y = pid_config.pid_rate_p_factor*p_value_y+pid_config.pid_rate_i_factor*i_value_rate_y+pid_config.pid_rate_d_factor*d_value_y;
	 
	 // Remember the last error to calc next values
	 last_rate_error_x = error_x;
	 last_rate_error_y = error_y;
 }
 
static void pid_vertical_velocity_step(float vertical_velocity, float elapsed_seconds) {
	 float error, p_value, d_value;
	 
	 //Calc error
	 error = target_vertical_velocity-vertical_velocity;

	 // P Value is the error directly
	 p_value = error;
	 
	 // I Value is the discrete integral (sum) of the product of error and time
	 i_value_vertical_velocity += elapsed_seconds * error;
	 
	 // D Value is the change of error
	 d_value = (error-last_vertical_velocity_error)/elapsed_seconds;
	 
	 // Generate pid value by weighted sum
	 pid_vertical_velocity_value = pid_config.pid_vertical_velocity_p_factor*p_value + pid_config.pid_vertical_velocity_i_factor*i_value_vertical_velocity + pid_config.pid_vertical_velocity_d_factor*d_value;
	 
	 // Remember the last error to calc next values
	 last_vertical_velocity_error = error;
 }
 
 void pid_step(float x_angle, float y_angle, float x_rate, float y_rate, float vertical_velocity) {
	 float elapsed_seconds = elapsed_time_s(last_pid_tick);
	 last_pid_tick = current_ticks();
	 if(elapsed_seconds > 1.0f) {
		 elapsed_seconds = 1.0f;
	 }
	 if(elapsed_seconds > 0) {
		 pid_called();
		 pid_angle_step(x_angle, y_angle, elapsed_seconds);
		 pid_rate_step(x_rate, y_rate, elapsed_seconds);
		 pid_vertical_velocity_step(vertical_velocity, elapsed_seconds);
	 }
 }
 
 void set_target(float angleX, float angleY) {
	 target_angle_x = angleX;
	 target_angle_y = angleY;
 }
 
 void set_target_vertical_velocity(float vertical_velocity) {
	 target_vertical_velocity = vertical_velocity;
 }
 
 
