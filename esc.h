#ifndef ESC_INO
#define ESC_INO

#define ESC_HIGH 2000
#define ESC_LOW 1000

void minThrottle(void);
void maxThrottle(void);
void setupESC(void);
void axisTest(void);
void setupThrottleRange(void);

#endif //ESC_INO