/*
 * controller.c
 *
 * Created: 27.10.2018 10:46:17
 *  Author: xeedn
 */ 
#include "controller.h"
#include "i2c.h"
#include "pid.h"
#include "esc.h"

void setup_controller(Twi* interface) {
	printf("Controller setup...\n");
	controller_interface = interface;
	openI2CClient(controller_interface, SLAVE_ADDRESS, &on_receive);
	printf("Controller setup done.\n");
	ticks = 0;
	last_measure = 0;
	last_control_ticks = 0;
	address = 6;
}

void on_receive(uint8_t* buffer, uint16_t count) {
	printf("Received a count of %d\n", count);
	if(count > 0) {
		printf("Received: '");
		for(uint32_t i=0; i<count;i++) {
			printf("%d ", buffer[i]);
		}
		printf("'\n");
		
		int16_t values[6];
		for(int i=0;i<6 && (i+1)*2 < count;i++) {
			values[i] = ((buffer[1+i*2] << 8) | buffer[2+i*2]);
		}
		if(buffer[0] == 0 && count == 7) {
			float ay = (float)(values[0])/MAX_VALUE;
			float ax = (float)(values[1])/MAX_VALUE;
			float th = (float)(values[2])/MAX_VALUE;
			printf("Decoded Control: %d %d %d\n", (int)(ax*100), (int)(ay*100), (int)(th*100));
			//Always apply target angles
			set_target(ax,ay);
			handleThrust(th);
		} else if(buffer[0] == 1 && count == 7) {
			float p = (float)(values[0])/MAX_VALUE;
			float i = (float)(values[1])/MAX_VALUE;
			float d = (float)(values[2])/MAX_VALUE;
			printf("Decoded PID: %d %d %d\n", (int)(p*1000), (int)(i*1000), (int)(d*1000));
			set_constants(p,i,d);
		} else if(buffer[0] == 2 && count == 7) {
			printf("Decoded Motor: %d %d %d\n", values[0], values[1], values[2]);
			adjustMotorValues(values[0], values[1], values[2]);
		} else if(buffer[0] == 3 && count == 13) {
			float p = (float)(values[0])/MAX_VALUE;
			float i = (float)(values[1])/MAX_VALUE;
			float d = (float)(values[2])/MAX_VALUE;
			printf("Decoded PID: %d %d %d\n", (int)(p*1000), (int)(i*1000), (int)(d*1000));
			printf("Decoded Motor: %d %d %d\n", values[3], values[4], values[5]);
			set_constants(p,i,d);
			adjustMotorValues(values[3], values[4], values[5]);
		} else if(buffer[0] == 10 && count == 1) {
			printf("Start received.\n");
			handleStart();
		} else if(buffer[0] == 11 && count == 1) {
			printf("Landing received.\n");
			handleLanding();
		} else if(buffer[0] == 12 && count == 1) {
			printf("Shutdown received.\n");
			handleShutdown();
		} else {
			printf("Could not decode message.\n");
			return;
		}
		last_control_ticks = ticks;
	}
}

void adjustMotorValues(uint16_t hover, uint16_t max, uint16_t landing) {
	HoverSpeed = hover;
	MaxSpeed = max;
	LandingSpeed = landing;
}

void handleStart(void) {
	if(state == IDLE_STATE || state == SHUTDOWN_STATE) {
		next_state = RUN_STATE;	
	}
}

void handleLanding(void) {
	if(state == RUN_STATE) {
		next_state = LANDING_STATE;
	}
}

void handleShutdown(void) {
	next_state = SHUTDOWN_STATE;
}


void handleThrust(float th) {
	if(state == RUN_STATE || state == LANDING_STATE) {
		int maxIncrease = MaxSpeed - HoverSpeed;
		int maxDecrease = HoverSpeed - ESC_LOW;
		if(th > 0) {
			BaseSpeed = HoverSpeed + (maxIncrease * th);
		} else {
			//th < 0, maxDecrease > 0 -> decrease
			BaseSpeed = HoverSpeed + (maxDecrease * th);
		}
		printf("New BaseSpeed: %d\n", BaseSpeed);
	} else {
		printf("Ignored Thrust\n");
	}
}