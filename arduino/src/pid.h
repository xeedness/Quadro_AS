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
	float z;
} pid_values_t;

enum Mode {
	MODE_ROTATION,
	MODE_HYBRID,
	MODE_ORIENTATION
};

extern pid_values_t pid_angle_values;
extern pid_values_t pid_rate_values;
extern float pid_vertical_velocity_value;

void pid_init(void);
void set_target(float angle_x, float angle_y);
void get_target(float* angle_x, float* angle_y);
void set_target_vertical_velocity(float vertical_velocity);
float get_target_vertical_velocity(void);
void pid_step(float x_angle, float y_angle, float x_rate, float y_rate, float z_rate, float vertical_velocity);
void set_mode(uint8_t in_mode);
uint8_t get_mode(void);

#endif /* PID_H_ */