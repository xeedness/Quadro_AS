/*
 * pid.h
 *
 * Created: 14.10.2018 18:16:57
 *  Author: xeedn
 */ 



#ifndef PID_H_
#define PID_H_

#include <asf.h>

typedef struct pid_values_s {
	float x;
	float y;	
} pid_values_t;

extern pid_values_t pid_angle_values;
extern pid_values_t pid_rate_values;

void pid_init(void);
void set_target(float angleX, float angleY);
void pid_step(float x_angle, float y_angle, float x_rate, float y_rate);

#endif /* PID_H_ */