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
float i_value_x;
float i_value_y;

float target_x, target_y;

uint32_t last_angle_tick;

typedef struct pid_values_s {
	float x;
	float y;	
} pid_values_t;

extern pid_values_t pid_values;

void pid_init(void);
void set_target(float angleX, float angleY);
void feed_angles(float angleX, float angleY);

#endif /* PID_H_ */