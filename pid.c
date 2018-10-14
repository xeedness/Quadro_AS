/*
 * pid.c
 *
 * Created: 14.10.2018 18:17:09
 *  Author: xeedn
 */ 

#include "pid.h"
#include <string.h>
 void init_pid() {
	 memset(angles_x, 0, HISTORY_SIZE*sizeof(float));
	 memset(angles_y, 0, HISTORY_SIZE*sizeof(float));
	 pid_value_x = 0;
	 pid_value_y = 0;
	 target_x = 0;
	 target_y = 0;
 }
 
 void set_constants(float p_k, float i_k, float d_k) {
	k1 = p_k;
	k1 = i_k;
	k1 = d_k; 
 }
 
 void pid_values(float* x, float* y) {
	 (*x) = pid_value_x;
	 (*y) = pid_value_y;
 }
 
 void feed_angles(float angleX, float angleY) {
	 
	 //Fill history
	 for(int i=HISTORY_SIZE-1;i>0;i--) {
		 angles_x[i+1] = angles_x[i];
		 angles_y[i+1] = angles_y[i];
	 }
	 //Calc error
	 angles_x[0] = angleX-target_x;
	 angles_y[0] = angleY-target_y;
	 
	 
	 float p_value_x, i_value_x, d_value_x;
	 float p_value_y, i_value_y, d_value_y;
	 
	 p_value_x = angles_x[0];
	 p_value_y = angles_y[0];
	 i_value_x = 0;
	 i_value_y = 0;
	 
	 for(int i=0;i<HISTORY_SIZE;i++) {
		i_value_x += SAMPLE_TIME * angles_x[i];
		i_value_y += SAMPLE_TIME * angles_y[i];
	 }
	 
	 
	 d_value_x = (angles_x[0]-angles_x[1])/SAMPLE_TIME;
	 d_value_y = (angles_y[0]-angles_y[1])/SAMPLE_TIME;
	 
	 pid_value_x = k1*p_value_x+k2*i_value_x+k3*d_value_x;
	 pid_value_y = k1*p_value_y+k2*i_value_y+k3*d_value_y; 
 }
 
 void set_target(float angleX, float angleY) {
	 target_x = angleX;
	 target_y = angleY;
 }
 
 
