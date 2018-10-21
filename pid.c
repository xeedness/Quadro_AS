/*
 * pid.c
 *
 * Created: 14.10.2018 18:17:09
 *  Author: xeedn
 */ 

#include "pid.h"
 void init_pid() {
	 pid_value_x = pid_value_y = 0;
	 target_x = target_y = 0;
	 i_value_x = i_value_y = 0;
	 last_error_x = last_error_y = 0;
 }
 
 void set_constants(float p_k, float i_k, float d_k) {
	k1 = p_k;
	k2 = i_k;
	k3 = d_k; 
 }
 
 void pid_values(float* x, float* y) {
	 (*x) = pid_value_x;
	 (*y) = pid_value_y;
 }
 
 void feed_angles(float angleX, float angleY) {
	 float error_x, p_value_x, d_value_x;
	 float error_y, p_value_y, d_value_y;
	 
	 //Calc error
	 error_x = angleX-target_x;
	 error_y = angleY-target_y; 
	 
	 // P Value is the error directly
	 p_value_x = error_x;
	 p_value_y = error_y;
	 
	 // I Value is the discrete integral (sum) of the product of error and time
	 i_value_x += SAMPLE_TIME * error_x;
	 i_value_y += SAMPLE_TIME * error_y;	 
	 
	 // D Value is the change of error
	 d_value_x = (error_x-last_error_x)/SAMPLE_TIME;
	 d_value_y = (error_y-last_error_y)/SAMPLE_TIME;
	 
	 // Generate pid value by weighted sum
	 pid_value_x = k1*p_value_x+k2*i_value_x+k3*d_value_x;
	 pid_value_y = k1*p_value_y+k2*i_value_y+k3*d_value_y; 
	 
	 // Remember the last error to calc next values
	 last_error_x = error_x;
	 last_error_y = error_y;
 }
 
 void set_target(float angleX, float angleY) {
	 target_x = angleX;
	 target_y = angleY;
 }
 
 
