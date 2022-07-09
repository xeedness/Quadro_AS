/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <stdio.h>

#include "esc.h"
#include "mpu9250.h"
#include "ibus.h"

void setup(void);
void forceMeasurement(void);

int main (void) {
    setup();
	printf("Main Loop\n");
    //forceMeasurement();
}

void setup(void) {
	sysclk_init();
	board_init();
	delay_init(sysclk_get_cpu_hz());
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	//Enable sensor interrupt
	pmc_enable_periph_clk(ID_PIOB);
	// Component initialization
	//setupESC();	
	
	delay_s(2);
	
	ibus_setup();
	ibus_loop();
	
	/*if(mpu9250_setup() == 0) {
		mpu9250_loop();
	}*/
	
	
	
	delay_s(1);
	printf("Setup finished");
	
	
}

void forceMeasurement(void) {
	while(1) {
		minThrottle();
		printf("Running Force Measurement. Rdy?\n");
		getchar();
		for(int percentage = 0; percentage <= 100; percentage+=10) {
			printf("Percent: %d\n", percentage);
			speed.front_left_speed = speed.front_right_speed = speed.rear_left_speed = speed.rear_right_speed = (float)(percentage)/100.0f;
			writeSpeed();
			getchar();
		}
		minThrottle();
		printf("Force Measurement done. Press any key to continue.\n");
		getchar();
	}
}