#ifndef ESC_H
#define ESC_H

#include <stdint.h>

#define ESC_HIGH 2000
#define ESC_LOW 1000

typedef struct speed {
	uint16_t front_left_speed;
	uint16_t front_right_speed;
	uint16_t rear_left_speed;
	uint16_t rear_right_speed;
} speed_t;

extern speed_t speed;

uint16_t current_base_speed;

// Core
void setupESC(void);
void setCurrentBaseSpeed(uint16_t base_speed);
void minThrottle(void);
void maxThrottle(void);
void writeSpeed(void);

// Unused
void axisTest(void);
void setupThrottleRange(void);

#endif //ESC_H