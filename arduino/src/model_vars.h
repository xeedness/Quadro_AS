/*
 * model_vars.h
 *
 * Created: 03.10.2020 10:52:31
 *  Author: xeedn
 */ 


#ifndef MODEL_VARS_H_
#define MODEL_VARS_H_

typedef struct angular_rate {
	float wx, wy, wz;
} angular_rate_t;

typedef struct orientation {
	float ax, ay, az;
} orientation_t;

typedef struct acceleration {
	float ax, ay, az;
} acceleration_t;

typedef struct altimeter {
	float altitude;
	float vertical_velocity;
	float vertical_acceleration;
} altimeter_t;

#endif /* MODEL_VARS_H_ */