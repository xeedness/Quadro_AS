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
#include "pid.h"

int LandingSpeed;
int HoverSpeed;
int MaxSpeed;
int CurrentSpeed;
int BaseSpeed;
int Step = 50;

//#define THROTTLE_SETUP
//#define SENSOR_AXIS_TEST
//#define AXIS_TEST

int dataCount = 1000;
#define STARTUP_TIME 5000
#define RUN_TIME 10000
#define LANDING_TIME 5000
#define PID_FACTOR 0.5f
#define MAX_FIFO_DATA 100

int state;
int next_state;
unsigned long int state_timer_ms;
int state_time_interval;
#define STARTUP_STATE 0
#define RUN_STATE 1
#define LANDING_STATE 2
#define SHUTDOWN_STATE 3

int front_left_speed;
int front_right_speed;
int rear_left_speed;
int rear_right_speed;

const float P_FACT = 0.4f;
const float I_FACT = 0.001f;
const float D_FACT = 0.0001f;

void updateSpeed(void);
void gotoState(int newState);
void delayState(int ms);
void onFifoFull(void);
void printSensorData(void);
void onDataReady(uint32_t arg0, uint32_t arg1);
void runAutomatic(void);
void setup(void);
void run(void);



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
	runAutomatic();
    //run();
	/*while(1) {
		delay_s(5);
		printf("Running\n");
		//onFifoFull();
	}*/
}  // end of main

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
	printf("Setup: Serial port communication at 9600bps\n");
	
	//Enable sensor interrupt
	pmc_enable_periph_clk(ID_PIOB);
	pio_set_output(PIOB, PIO_PB21, LOW, DISABLE, ENABLE);
	pio_clear(PIOB, PIO_PB21);
	pio_set_input(PIOB, PIO_PB26, PIO_PULLUP);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB26, PIO_IT_FALL_EDGE, onDataReady);
	pio_enable_interrupt(PIOB, PIO_PB26);
	NVIC_EnableIRQ(PIOB_IRQn);
	
	setupESC();
	
	delay_s(1);
	pio_set(PIOB, PIO_PB21);
	
	

    
    setupSensor();
	
	set_constants(P_FACT,I_FACT,D_FACT);
	set_target(0,0);
	BaseSpeed = 1000;
	LandingSpeed = 1200;
	HoverSpeed = 1350;
	MaxSpeed = 1800;
	
	
	printf("Interrupt Setup.\n");
	
	
	delay_s(1);
	printf("Setup completed.\n");
}
void gotoState(int newState) {
	state = newState;
	next_state = newState;
	state_timer_ms = 0;
}
void delayState(int ms) {
	delay_ms(ms);
	state_timer_ms += ms;
}

int outputCounter = 0;
void updateSpeed(void) {
	orientation_t orientation;
	getOrientation(&orientation);
	feed_angles(orientation.ax, orientation.ay);
	float pid_x, pid_y;
	pid_values(&pid_x, &pid_y);
	
	//Sum PID values with negated values for right and rear respectively
	front_left_speed = BaseSpeed + (pid_x/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW)) + (pid_y/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW));
	front_right_speed = BaseSpeed + (pid_x/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW)) + (-pid_y/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW));
	rear_left_speed = BaseSpeed + (-pid_x/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW)) + (pid_y/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW));
	rear_right_speed = BaseSpeed + (-pid_x/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW)) + (-pid_y/100.0f * PID_FACTOR * (ESC_HIGH-ESC_LOW));
	
	front_left_speed = min(front_left_speed, MaxSpeed);
	front_right_speed = min(front_right_speed, MaxSpeed);
	rear_left_speed = min(rear_left_speed, MaxSpeed);
	rear_right_speed = min(rear_right_speed, MaxSpeed);
	
	REG_PWM_CDTYUPD0 = front_left_speed;
	REG_PWM_CDTYUPD1 = front_right_speed;
	REG_PWM_CDTYUPD2 = rear_right_speed;
	REG_PWM_CDTYUPD3 = rear_left_speed;
	
	if(++outputCounter > 10) {
		outputCounter = 0;
		printf("PID Values: %d, %d\n", (int)pid_x, (int)pid_y);
		printf("Motor Values:\n");
		printf("%d %d\n%d %d\n", front_left_speed, front_right_speed, rear_left_speed, rear_right_speed);
		//position_t position;
		//getPosition(&position);
		//printf("Position: %d, %d, %d [cm]\n", (int)(position.x*100.0f), (int)(position.y*100.0f), (int)(position.z*100.0f));
	}
}


void runAutomatic(void) {
	state = -1;
	next_state = STARTUP_STATE;	
	while(1) {
		
		//Process current state
		switch(state) {
		case(STARTUP_STATE):
			printf("Starting in %lu s\n", (STARTUP_TIME-state_timer_ms)/1000);
			if(state_timer_ms >= STARTUP_TIME) {
				next_state = RUN_STATE;
			}
			break;
		case(RUN_STATE):
			updateSpeed();
			if(state_timer_ms >= RUN_TIME) {
				next_state = LANDING_STATE;
			}
			break;
		case(LANDING_STATE):
			if(BaseSpeed > LandingSpeed)
			{
				BaseSpeed--;
			}
			updateSpeed();
			if(state_timer_ms >= LANDING_TIME) {
				next_state = SHUTDOWN_STATE;
			}
			break;
		case(SHUTDOWN_STATE):
			printf("Stopped\n");
			break;
		default:
			printf("No State\n");
			break;
		}
		
		if(state != next_state) {
			//Process next state
			switch(next_state) {
			case(STARTUP_STATE):
				minThrottle();
				state_time_interval = 1000;
				break;
			case(RUN_STATE):
				printf("Running...");
				state_time_interval = 20;
				BaseSpeed = HoverSpeed;
				break;
			case(LANDING_STATE):
				printf("Landing...");
				state_time_interval = 20;
				//BaseSpeed = LandingSpeed;
				break;
			case(SHUTDOWN_STATE):
				printf("Stopping...");
				state_time_interval = 1000;
				minThrottle();
				break;
			}
			gotoState(next_state);
		} else {
			// Increase time, when no state change occurred
			delayState(state_time_interval);
		}
	}
}

void run(void) {
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
		delay_ms(100);
    }
    //}
}

void onFifoFull(void) {
	printf("Interrupt\n");
	accel_gyro_union data[MAX_FIFO_DATA];
	uint32_t count = getFifoSensorData(data, MAX_FIFO_DATA);
	printf("Received %d data points\n", (int)count);
	for(uint32_t i=0;i<count;i++) {
		printf("AcX = %d |  | AcY = %d | AcZ = %d | GyX = %d | GyY = %d | GyZ = %d\n",
		data[i].value.x_accel,
		data[i].value.y_accel,
		data[i].value.z_accel,
		data[i].value.x_gyro,
		data[i].value.y_gyro,
		data[i].value.z_gyro);
	}
}

void printSensorData(void) {
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

void onDataReady(uint32_t arg0, uint32_t arg1) {
	if(state != SHUTDOWN_STATE) {
		updateOrientation();
	}
}