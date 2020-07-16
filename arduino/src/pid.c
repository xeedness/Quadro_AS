/*
 * pid.c
 *
 * Created: 14.10.2018 18:17:09
 *  Author: xeedn
 */ 

#include "pid.h"
#include "timer.h"
#include "config.h"

pid_values_t pid_values = {0, 0};

 void pid_init(void) {
	 //pid_values = {0, 0};
	 target_x = target_y = 0;
	 i_value_x = i_value_y = 0;
	 last_error_x = last_error_y = 0;
	 last_angle_tick = current_ticks();
 }
 
 void feed_angles(float angleX, float angleY) {
	 float error_x, p_value_x, d_value_x;
	 float error_y, p_value_y, d_value_y;
	 
	 float elapsed_time = elapsed_time_s(last_angle_tick);
	 last_angle_tick = current_ticks();
	 if(elapsed_time > 1.0f) {
		 elapsed_time = 1.0f;
	 }
	 //printf("Elapsed Time: %.5f\n", elapsed_time);
	 
	 //Calc error
	 error_x = angleX-target_x;
	 error_y = angleY-target_y; 
	 
	 // P Value is the error directly
	 p_value_x = error_x;
	 p_value_y = error_y;
	 
	 // I Value is the discrete integral (sum) of the product of error and time
	 i_value_x += elapsed_time * error_x;
	 i_value_y += elapsed_time * error_y;	 
	 
	 // D Value is the change of error
	 d_value_x = (error_x-last_error_x)/elapsed_time;
	 d_value_y = (error_y-last_error_y)/elapsed_time;
	 
	 // Generate pid value by weighted sum
	 pid_values.x = pid_config.pid_p_factor*p_value_x+pid_config.pid_i_factor*i_value_x+pid_config.pid_d_factor*d_value_x;
	 pid_values.y = pid_config.pid_p_factor*p_value_y+pid_config.pid_i_factor*i_value_y+pid_config.pid_d_factor*d_value_y; 
	 
	 // Remember the last error to calc next values
	 last_error_x = error_x;
	 last_error_y = error_y;
 }
 
 void set_target(float angleX, float angleY) {
	 target_x = angleX;
	 target_y = angleY;
 }
 
 
