/*
 * controller.h
 *
 * Created: 27.10.2018 10:46:06
 *  Author: xeedn
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <asf.h>

#define SLAVE_ADDRESS 0x62
#define MAX_VALUE 32768.0f

#define IDLE_STATE 0
#define RUN_STATE 1
#define LANDING_STATE 2
#define SHUTDOWN_STATE 3
//#define STARTUP_TIME 5000
//#define RUN_TIME 10000
#define LANDING_TIME 10000
#define PID_FACTOR 0.5f
int state;
int next_state;

volatile uint32_t last_control_ticks;
volatile uint32_t ticks; //1 tick == 10 us
volatile uint32_t last_measure;

uint8_t address;
Twi* controller_interface;

uint16_t LandingSpeed;
uint16_t HoverSpeed;
uint16_t MaxSpeed;
uint16_t CurrentSpeed;
uint16_t BaseSpeed;

void setup_controller(Twi* interface);
void on_receive(uint8_t*, uint16_t);
void adjustMotorValues(uint16_t hover, uint16_t max, uint16_t landing);
void handleThrust(float th);
void handleStart(void);
void handleLanding(void);
void handleShutdown(void);


#endif /* CONTROLLER_H_ */