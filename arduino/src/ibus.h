/*
 * ibus.h
 *
 * Created: 6/7/2021 10:04:36 PM
 *  Author: xeedn
 */ 


#ifndef IBUS_H_
#define IBUS_H_

#include <stdint.h>

#define IBUS_CHANNEL_COUNT 10

void ibus_setup(void);
int ibus_read(uint8_t ch);
uint16_t* ibus_data(void);


#endif /* IBUS_H_ */