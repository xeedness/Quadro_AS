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
#include "conf_usart.h"
#include <stdio.h>
#include "esc.h"
#include "sensor.h"

int CurrentSpeed;
int Step = 50;

//#define THROTTLE_SETUP
//#define SENSOR_AXIS_TEST
//#define AXIS_TEST


#define MAX_FIFO_DATA 100
void onFifoFull() {
	printf("Interrupt\n");
    accel_gyro_union data[MAX_FIFO_DATA];
    uint32_t count = getFifoSensorData(data, MAX_FIFO_DATA);
    printf("Received %d data points\n", count); 
    for(int i=0;i<count;i++) {
		printf("AcX = %d |  | AcY = %d | AcZ = %d | Tmp = %.2 | GyX = %d | GyY = %d | GyZ = %d\n", 
			data[i].value.x_accel, 
			data[i].value.y_accel,
			data[i].value.z_accel,
			//data[i].Tmp/340.00+36.53,
			data[i].value.x_gyro,
			data[i].value.y_gyro,
			data[i].value.z_gyro);
    }
}

void printSensorData() {
	//printf("Interrupt\n");
	accel_t_gyro_union sensorData;
	if(getSensorData(&sensorData) != 0) {
		return;
	}
	printf("AcX = %d | AcY = %d | AcZ = %d | GyX = %d | GyY = %d | GyZ = %d\n",
	sensorData.value.x_accel,
	sensorData.value.y_accel,
	sensorData.value.z_accel,
	//sensorData.value.temperature/340.0f+36.53f,
	sensorData.value.x_gyro,
	sensorData.value.y_gyro,
	sensorData.value.z_gyro);
}

void setup() {
	sysclk_init();
	board_init();
	delay_init(sysclk_get_cpu_hz());
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	printf("Setup: Serial port communication at 9600bps\n");

    setupESC();
    //Enable interrupt
	pmc_enable_periph_clk(ID_PIOB);
	pio_set_input(PIOB, PIO_PB26, PIO_PULLUP);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB26, PIO_IT_FALL_EDGE, printSensorData);
	pio_enable_interrupt(PIOB, PIO_PB26);
	NVIC_EnableIRQ(PIOB_IRQn);
	//gpio_configure_pin(PIO_PA22_IDX, PIO_TYPE_PIO_INPUT | PIO_PULLUP)
    //pinMode(22, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(22), onFifoFull, CHANGE);
    setupSensor(); 
}

void run() {
    minThrottle();
    printf("Running ESC\n");
    printf("Step = %d\n", Step);
    printf("Press 'u' to increase speed, 'd' to reduce speed\n");
	
    CurrentSpeed = ESC_LOW;
    while (1) {
		char currentChar = '0';
		if(usart_serial_is_rx_ready((Usart *)CONF_UART)) {
			currentChar = getchar();
			printf("Got input.\n");
		}
        if (currentChar == 'u')
        {
            printf("\nIncreasing motor speed by step\n");
            if (CurrentSpeed + Step < ESC_HIGH) {
            CurrentSpeed = CurrentSpeed + Step;
			printf("New speed = %d", CurrentSpeed);
            }

            else
            {
            printf("\nMax speed reached\n");
            }
        }
        if (currentChar == 'i')
        {
            printf("\nIncreasing motor speed by step");
            if (CurrentSpeed + 5 < ESC_HIGH) {
            CurrentSpeed = CurrentSpeed + 5;
            printf("New speed = %d", CurrentSpeed);
            }

            else
            {
            printf("\nMax speed reached\n");
            }
        }

        if (currentChar == 'd')
        {
            printf("\nDecreasing motor speed by step\n");
            if (CurrentSpeed - Step >= ESC_LOW)
            {
            CurrentSpeed = CurrentSpeed - Step;
            printf("New speed = %d", CurrentSpeed);
            }

            else
            {
            printf("\nMin speed reached\n");
            }
        }
        if (currentChar == 'f')
        {
            printf("\nDecreasing motor speed by step\n");
            if (CurrentSpeed - 5 >= ESC_LOW)
            {
            CurrentSpeed = CurrentSpeed - 5;
            printf("New speed = %d", CurrentSpeed);
            }

            else
            {
            printf("\nMin speed reached\n");
            }
        }
        if (currentChar == 'e')
        {
            printf("\nStopping Motors\n");
            CurrentSpeed = ESC_LOW;
        }
        REG_PWM_CDTYUPD0 = REG_PWM_CDTYUPD1 = REG_PWM_CDTYUPD2 = REG_PWM_CDTYUPD3 = CurrentSpeed;
		printSensorData();
		delay_ms(333);
        }
    //}
}

int main (void) {
    setup();
#ifdef SENSOR_AXIS_TEST
    sensorAxisTest();
#endif
#ifdef THROTTLE_SETUP
    setupThrottleRange(); 
#endif
#ifdef AXIS_TEST
    axisTest();
#endif
    run();
	/*while(1) {
		delay_s(2);
		printf("Running\n");
		//onFifoFull();
	}*/
}  // end of main

