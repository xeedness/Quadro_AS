#ifndef ESC_H
#define ESC_H

#include <stdint.h>

#define ESC_HIGH 2000
#define ESC_LOW 1000

typedef struct speed {
	float front_left_speed;
	float front_right_speed;
	float rear_left_speed;
	float rear_right_speed;
} speed_t;

extern speed_t speed;

// Core
void setupESC(void);
void minThrottle(void);
void maxThrottle(void);
void writeSpeed(void);

// Unused
void axisTest(void);
void setupThrottleRange(void);

#endif //ESC_H