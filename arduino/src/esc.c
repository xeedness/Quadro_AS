/*
 * esc.c
 *
 * Created: 15.10.2018 19:25:15
 *  Author: xeedn
 */ 

#include "esc.h"
#include "asf.h"

//! Array containing the throttle percentage (MIN_THROTTLE, MAX_THROTTLE)
speed_t speed;

void setupESC(void) {
	// PWM Set-up on pin: DAC1
	REG_PMC_PCER1 |= PMC_PCER1_PID36;                     // Enable PWM
	REG_PIOC_ABSR |= PIO_ABSR_P2;                        // Set PWM pin perhipheral type A or B, in this case B
	REG_PIOC_PDR |= PIO_PDR_P2;                          // Set PWM pin to an output
	REG_PIOC_ABSR |= PIO_ABSR_P4;                        // Set PWM pin perhipheral type A or B, in this case B
	REG_PIOC_PDR |= PIO_PDR_P4;                          // Set PWM pin to an output
	REG_PIOC_ABSR |= PIO_ABSR_P6;                        // Set PWM pin perhipheral type A or B, in this case B
	REG_PIOC_PDR |= PIO_PDR_P6;                          // Set PWM pin to an output
	REG_PIOC_ABSR |= PIO_ABSR_P8;                        // Set PWM pin perhipheral type A or B, in this case B
	REG_PIOC_PDR |= PIO_PDR_P8;                          // Set PWM pin to an output
	REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);     // Set the PWM clock rate to 2MHz (84MHz/42)
	REG_PWM_CMR0 = REG_PWM_CMR1 = REG_PWM_CMR2 = REG_PWM_CMR3 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
	REG_PWM_CPRD0 = REG_PWM_CPRD1 = REG_PWM_CPRD2 = REG_PWM_CPRD3 = 20000;                                // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz
	REG_PWM_CDTY0 = REG_PWM_CDTY1 = REG_PWM_CDTY2 = REG_PWM_CDTY3 = ESC_LOW;                              // Set the PWM duty cycle to 1500 - centre the servo
	REG_PWM_ENA = PWM_ENA_CHID0 | PWM_ENA_CHID1 | PWM_ENA_CHID2 | PWM_ENA_CHID3;                          // Enable the PWM channel
	
	minThrottle();
	printf("ESC Setup done\n");
}

void minThrottle(void) {
	speed.front_left_speed = speed.front_right_speed = speed.rear_left_speed = speed.rear_right_speed = 0;
	writeSpeed();
}

void maxThrottle(void) {
	speed.front_left_speed = speed.front_right_speed = speed.rear_left_speed = speed.rear_right_speed = 1;
	writeSpeed();
}

void writeSpeed(void) {
	uint16_t fl = ESC_LOW + (ESC_HIGH-ESC_LOW) * speed.front_left_speed;
	uint16_t fr = ESC_LOW + (ESC_HIGH-ESC_LOW) * speed.front_right_speed;
	uint16_t rl = ESC_LOW + (ESC_HIGH-ESC_LOW) * speed.rear_left_speed;
	uint16_t rr = ESC_LOW + (ESC_HIGH-ESC_LOW) * speed.rear_right_speed;
	
	REG_PWM_CDTYUPD0 = fl;
	REG_PWM_CDTYUPD1 = fr;
	REG_PWM_CDTYUPD2 = rl;
	REG_PWM_CDTYUPD3 = rr;
	//printf("Written Speed Values: %u %u %u %u\n", fl, fr, rl, rr);
}


void axisTest(void) {
	while(1) {
		minThrottle();
		printf("Running Axis Test. Rdy?\n");
		getchar();
		printf("Spinning Left\n");
		REG_PWM_CDTYUPD0 = REG_PWM_CDTYUPD2 = 1100;
		getchar();
		minThrottle();

		getchar();
		printf("Spinning Right\n");
		REG_PWM_CDTYUPD1 = REG_PWM_CDTYUPD3 = 1100;
		getchar();
		minThrottle();

		getchar();
		printf("Spinning Back\n");
		REG_PWM_CDTYUPD2 = REG_PWM_CDTYUPD3 = 1100;
		getchar();
		minThrottle();

		getchar();
		printf("Spinning Front\n");
		REG_PWM_CDTYUPD0 = REG_PWM_CDTYUPD1 = 1100;
		getchar();
		minThrottle();

		printf("Axis Test done. Press any key to continue\n");
		getchar();
	}
}

void setupThrottleRange(void) {
	printf("In Set Throttle Range mode\n");
	maxThrottle();

	printf("Connect the ESC now. After connecting the ESC, you should hear the ESC startup tones. Shortly afterwards, you should hear two beeps indicating that the ESC has registered the high throttle value. Immediately after hearing the two beeps, push any key. If you don't do so in 5 sec, the ESC will go into program mode\n");
	getchar();

	printf("\nSetting the low throttle setting. If this happens successfully, you should hear several beeps indicating the input voltage supplied to the ESC followed by a long beep indicating that the low throttle has been set. After this point, push any key to proceed, your ESC is ready to be used\n");

	minThrottle();
	getchar();
	
}