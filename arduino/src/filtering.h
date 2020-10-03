/*
 * filtering.h
 *
 * Created: 24.09.2020 16:34:44
 *  Author: xeedn
 */ 


#ifndef FILTERING_H_
#define FILTERING_H_

#include "sensor.h"
#include "model_vars.h"

void setupKalman(void);
void getFilteredOrientation(orientation_t* phi, angular_rate_t* omega);
void updateOrientationFilter(void);
void getFilteredAltitude(altimeter_t* altimeter_data);
void updateAltitudeFilter(void);

#endif /* FILTERING_H_ */