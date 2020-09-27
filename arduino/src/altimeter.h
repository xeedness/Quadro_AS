/*
 * altimeter.h
 *
 * Created: 25.09.2020 17:39:59
 *  Author: xeedn
 */ 


#ifndef ALTIMETER_H_
#define ALTIMETER_H_

#include <asf.h>

void setupAltimeter(Twi* interface);
uint8_t updateAltitude(void);
double getAltitude(void);
uint32_t getAltitudeTick(void);



#endif /* ALTIMETER_H_ */