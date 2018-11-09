/*
 * pid.h
 *
 * Created: 14.10.2018 18:16:57
 *  Author: xeedn
 */ 



#ifndef PID_H_
#define PID_H_

#include <asf.h>

#define SAMPLE_RATE 50
//#define HISTORY_SECONDS 10
//#define HISTORY_SIZE SAMPLE_RATE*HISTORY_SECONDS
#define K_P 1

//float angles_x[HISTORY_SIZE];
//float angles_y[HISTORY_SIZE];
float last_error_x;
float last_error_y;
float k1, k2, k3;
float pid_value_x;
float pid_value_y;
float i_value_x;
float i_value_y;

float target_x, target_y;

uint32_t last_angle_tick;

void init_pid(void);
void set_constants(float p_k, float i_k, float d_k);
void set_target(float angleX, float angleY);
void feed_angles(float angleX, float angleY);
void pid_values(float* x, float* y);

#endif /* PID_H_ */