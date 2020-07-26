/*
 * controller.h
 *
 * Created: 27.10.2018 10:46:06
 *  Author: xeedn
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <asf.h>

void setup_controller(void);
bool execute_command(uint8_t cmd, uint8_t* payload);

void update_speed(void);

uint8_t request_init(uint8_t is_repeat);
uint8_t request_init_status(void);

uint8_t is_controller_alive(void);

#endif /* CONTROLLER_H_ */