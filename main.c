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
#include "controller.h"



//#define THROTTLE_SETUP
//#define SENSOR_AXIS_TEST
//#define AXIS_TEST

uint8_t sensor_data_ready = 0;
uint32_t last_state_run_ticks = 0;
uint32_t log_begin_ticks = 0;
uint32_t last_second_tick = 0;


uint32_t state_timer_us;
uint32_t state_time_interval_ms;

uint16_t front_left_speed;
uint16_t front_right_speed;
uint16_t rear_left_speed;
uint16_t rear_right_speed;

const float P_FACT = 0.1f;
const float I_FACT = 0.01f;
const float D_FACT = 0.01f;

void updateSpeed(void);
void gotoState(int newState);
void delayState(uint32_t ms);
void printSensorData(void);
void onDataReady(uint32_t arg0, uint32_t arg1);
void runControl(void);
void runLog(void);
void setup(void);


float angleLogX[4000];
float angleLogY[4000];
float pidLogX[4000];
float pidLogY[4000];
uint16_t logSize = 0;

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
	//runControl();
	runLog();
}  // end of main



void setup(void) {
	sysclk_init();
	board_init();
	delay_init(sysclk_get_cpu_hz());
	
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		while (true) {  /* no error must happen here, otherwise this board is dead */ }
	}
	
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
	NVIC_SetPriority(PIOB_IRQn, 1);
	NVIC_EnableIRQ(PIOB_IRQn);
	
	setupESC();
	
	delay_s(1);
	pio_set(PIOB, PIO_PB21);
	
	

    
    setupSensor(TWI1);
	setup_controller(TWI0);
	
	set_constants(P_FACT,I_FACT,D_FACT);
	set_target(0,0);
	BaseSpeed = 1000;
	LandingSpeed = 1200;
	HoverSpeed = 1350;
	MaxSpeed = 1800;
	MinSpeed = 1100;
	
	
	printf("Interrupt Setup.\n");
	
	
	delay_s(1);
	printf("Setup completed.\n");
}

void SysTick_Handler(void)
{
	ticks++;
}

void gotoState(int newState) {
	state = newState;
	next_state = newState;
	state_timer_us = 0;
}


/*void delayState(uint32_t us) {
	uint32_t past_us = elapsed_time_us(last_measure);
	//If delay already passed. Just wait full time, but add it to the state timer
	if(past_us > us) {
		delay_us(us);	
		state_timer_us += us + past_us;
	} else {
		delay_ms(us - past_us);	
		state_timer_us += us;
	}
	last_measure = ticks;
}*/

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
	
	front_left_speed = max(front_left_speed, MinSpeed);
	front_right_speed = max(front_right_speed, MinSpeed);
	rear_left_speed = max(rear_left_speed, MinSpeed);
	rear_right_speed = max(rear_right_speed, MinSpeed);
	
	
	
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
uint8_t began = 0;
void runLog(void) {
	
	last_second_tick = ticks;
	while(1) {
		if(sensor_data_ready) {
			updateOrientation();
			sensor_data_ready = 0;
			if(began == 0) {
				began = 1;	
				log_begin_ticks = ticks;
			}
			
		}
		if(began && elapsed_time_ms(last_state_run_ticks) > 100) {
			orientation_t orientation;
			getOrientation(&orientation);
			feed_angles(orientation.ax, orientation.ay);
			float pid_x, pid_y;
			pid_values(&pid_x, &pid_y);
			angleLogX[logSize] = orientation.ax;
			angleLogY[logSize] = orientation.ay;
			pidLogX[logSize] = pid_x*PID_FACTOR/100.0f;
			pidLogY[logSize] = pid_y*PID_FACTOR/100.0f;
			logSize++,
			last_state_run_ticks = ticks;
		}
		if(elapsed_time_s(last_second_tick) > 1) {
			last_second_tick = ticks;
			printf("Second...\n");
		}
		if(began && elapsed_time_s(log_begin_ticks) > 20) {
			break;
		}
		delay_us(1);
	}
	printf("Logging %d values\n", logSize);
	for(int i=0;i<logSize;i++) {
		printf("%.4f;%.4f;%.4f;%.4f\n", angleLogX[i], angleLogY[i], pidLogX[i], pidLogY[i]);
	}
}

void runControl(void) {
	state = -1;
	next_state = IDLE_STATE;	
	while(1) {
		if(sensor_data_ready) {
			updateOrientation();
			sensor_data_ready = 0;
		}
		//printf("Elapsed US: %lu\n", elapsed_time_us(last_state_run_ticks));
		if(elapsed_time_ms(last_state_run_ticks) > state_time_interval_ms) {
			//Process current state
			switch(state) {
			case(IDLE_STATE):
				printf("Idle\n");
				printf("Valid: %lu Invalid: %lu\n", valid, invalid);
				orientation_t orientation;
				getOrientation(&orientation);
				printf("AngleX: %d AngleY: %d\n", (int16_t)orientation.ax, (int16_t)orientation.ay);
				break;
			case(RUN_STATE):
				updateSpeed();
				if(elapsed_time_s(last_control_ticks) > 10) {
					next_state = LANDING_STATE;
				}
				break;
			case(LANDING_STATE):
				if(BaseSpeed > LandingSpeed)
				{
					BaseSpeed--;
				}
				if(CurrentLandingSpeed > LandingSpeed)
				{
					CurrentLandingSpeed--;
				}				
				updateSpeed();
				if(state_timer_us >= LANDING_TIME*100) {
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
				case(IDLE_STATE):
					minThrottle();
					state_time_interval_ms = 1000;
					break;
				case(RUN_STATE):
					printf("Running...");
					state_time_interval_ms = 20;
					BaseSpeed = HoverSpeed;
					break;
				case(LANDING_STATE):
					printf("Landing...");
					state_time_interval_ms = 20;
					CurrentLandingSpeed = HoverSpeed;
					break;
				case(SHUTDOWN_STATE):
					printf("Stopping...");
					state_time_interval_ms = 1000;
					minThrottle();
					break;
				}
				gotoState(next_state);
			}
			last_state_run_ticks = ticks;
		}
		// Increase time, when no state change occurred
		delay_us(1);
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
		sensor_data_ready = 1;
	}
}



/*
int Step = 50;

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
}*/