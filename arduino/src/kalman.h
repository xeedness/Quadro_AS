/*
 * kalman.h
 *
 * Created: 24.09.2020 16:34:44
 *  Author: xeedn
 */ 


#ifndef KALMAN_H_
#define KALMAN_H_

#include "sensor.h"

typedef struct altimeter {
	float altitude;
	float vertical_velocity;
	float vertical_acceleration;
} altimeter_t;

void setupKalman(void);
void getKalmanOrientationEstimate(orientation_t* phi, angular_rate_t* omega);
void updateKalmanOrientation(void);
void getKalmanAltituteEstimate(altimeter_t* altimeter_data);
void updateKalmanAltitude(void);

#endif /* KALMAN_H_ */