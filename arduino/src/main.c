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
#include "altimeter.h"
#include "pid.h"
#include "controller.h"
#include "timer.h"
#include "config.h"
#include "state_machine.h"
#include "status_display.h"
#include "call_counter.h"
#include "filtering.h"
#include "ibus.h"

#include "log.h"
#include "uart_bridge.h"

#include <string.h>
#include "persistent_mem.h"


void setup(void);
void set_default_config(void);
void timed_pid_update(void);
void timed_logging(void);
void timed_speed_update(void);
void throttle_calibration(void);

pid_config_t pid_config;
esc_config_t esc_config;
log_config_t log_config;
sensor_config_t sensor_config;

uint32_t last_log_ticks = 0;

int main (void) {
    setup();
	reset_state();
	printf("Main Loop\n");
	/*setupThrottleRange();
	return 0;*/
	
	/* Kleiner Memtest 4 ze fun */
	char data[2048];
	read_data((uint8_t*)data, sizeof(data));
	printf("Persistent mem contents:\n");
	printf(data);
	
	const char str[] = "HERPA DERPA WOLOLOLO I BIMS 1 MIRCOCONTROLLER !!!11\n"; 
	uint32_t res = store_data( (uint8_t*)str, strlen(str)+1);
    /* Kleiner Memtest 4 ze fun ende */
	
    //axisTest();
	
	/*while(1) {
		//printf("Restart Please\n");
		delay_s(5);
	}*/
	
	uint32_t used_altitude_altimeter_tick = getAltitudeTick();
	uint32_t used_altitude_sensor_tick = getSensorTick();
	uint32_t used_attitute_sensor_tick = getSensorTick();
	
	uint32_t current_altimeter_tick = 0;
	uint32_t current_sensor_tick = 0;
	while(1) {
		// Always update orientation, if data is available here. The goal should be to use every sensor update. Currently not sure if that is given
		// Maybe consider moving this to the interrupt handler, since this is the most time critical thing
		
		current_altimeter_tick = getAltitudeTick();
		current_sensor_tick = getSensorTick();
		
		if(current_altimeter_tick != used_altitude_altimeter_tick && current_sensor_tick != used_altitude_sensor_tick) {
			updateAltitudeFilter();
		}
		
		if(current_sensor_tick != used_attitute_sensor_tick) {
			updateOrientationFilter();
		}		
		
		// Do all the time depending steps
		//timed_pid_update();
		//timed_logging();
		timed_state_tasks();
		process_call_counters();
		
		
		// Progress state machine
		state_transition();
		
		// Wait an arbitrary small enough time to do the next loop (considerations mainly depend on sensor update frequency)
		//delay_us(25);
	} 
	
	
	// TODO Controller trigger for sensor axis test
    //sensorAxisTest();
	// TODO Controller trigger for throttle range setup
    //setupThrottleRange(); 
	// TODO Controller trigger for rotor axis test
    //axisTest();
	
}

void setup(void) {
	sysclk_init();
	board_init();
	delay_init(sysclk_get_cpu_hz());
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		while (true) {  /* no error must happen here, otherwise this board is dead */ }
	}
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_CONSOLE_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);	
	
	// Component initialization
	set_default_config();
	controller_init(ENABLE_ESP_THROTTLE | ENABLE_ESP_ROTATION | ENABLE_ESP_POWER | ENABLE_IBUS_THROTTLE | ENABLE_IBUS_ROTATION | ENABLE_IBUS_POWER | ENABLE_IBUS_FLIGHTMODE);
	timer_init();
	pid_init();
	ibus_setup();
	uart_bridge_init();
	status_display_init();
	
	
	//setupAltimeter(TWI0);
	delay_s(1);
	setupESC();
	setupSensor(TWI1, current_ticks());
	setupKalman();
	
	reset_state();
	init_call_counter();
	
	last_log_ticks = current_ticks();
	
	delay_s(1);
	
	printf("Setup finished");
}

void SysTick_Handler(void)
{
	increment_state_ticks();
}

void set_default_config(void) {
	esc_config.min_speed = 0.1f;
	esc_config.max_speed = 0.8f;
	esc_config.vertical_velocity_pid_amplifier = 1.0f;
	esc_config.vertical_velocity_limit = 0.1f;
	esc_config.update_interval_ms = 20;
	
	pid_config.pid_amplify_factor = 0;
	pid_config.pid_angle_p_factor = 0.3f;
	pid_config.pid_angle_i_factor = 0.01f;
	pid_config.pid_angle_d_factor = 0.01f;
	pid_config.pid_rate_p_factor = 0.3f;
	pid_config.pid_rate_i_factor = 0.01f;
	pid_config.pid_rate_d_factor = 0.01f;
	pid_config.pid_vertical_velocity_d_factor = 1.0f;
	pid_config.pid_vertical_velocity_i_factor = 0.2f;
	pid_config.pid_vertical_velocity_p_factor = 0;
	pid_config.update_interval_ms = 20;
	
	log_config.orientation_enabled = 0;
	log_config.pid_enabled = 0;
	log_config.speed_enabled = 0;
	log_config.altitude_enabled = 0;
	log_config.log_interval_ms = 50;
	
	sensor_config.acceleration_weight = 0.1f;
	sensor_config.measurement_error_angle = 0.1f;
	sensor_config.measurement_error_angular_velocity = 0.001f;
	sensor_config.estimate_error_angle = 0.001f;
	sensor_config.estimate_error_angular_velocity = 0.1f;
	sensor_config.altitude_gain = -1.1547f; //-0.2828f; //-0.6325f;//-0.1925f;
	sensor_config.speed_gain = -0.6667; //-0.2f;//-0.0185f;
	sensor_config.use_kalman_orientation = 0;
	sensor_config.enabled = 1;
}

// Update PID values according to configured update interval
//void timed_pid_update(void) {
	//if(elapsed_time_ms(last_pid_ticks) > pid_config.update_interval_ms) {
		//printf("Updating angles: %.2f %.2f\n", current_orientation.ax, current_orientation.ay);
		//feed_angles(current_orientation.ax, current_orientation.ay);
		//last_pid_ticks = current_ticks();
	//}
//}

uint32_t last_uart_ticks = 0;
// Log PID values according to configured update interval
void timed_logging(void) {
	if(elapsed_time_ms(last_log_ticks) > log_config.log_interval_ms) {
		if(log_config.orientation_enabled) {			
			orientation_t cur_orientation;
			angular_rate_t cur_angular_velocity;
			getFilteredOrientation(&cur_orientation, &cur_angular_velocity);
			
			// Substitute Z Orientation with z acceleration for debugging purposes
			cur_orientation.ax /= DEG_TO_RAD_FACTOR;
			cur_orientation.ay /= DEG_TO_RAD_FACTOR;
			
			// Log target instead of orientation
			get_target(&cur_orientation.ax, &cur_orientation.ay);
			log_orientation(cur_orientation);
			log_angular_velocity(cur_angular_velocity);
		}
		
		if(log_config.pid_enabled) {
			//printf("PID Logging: %.2f %.2f\n", pid_rate_values.x, pid_rate_values.y);
			log_pid(pid_rate_values);
		}
		
		if(log_config.speed_enabled) {
			//printf("Speed Logging: %d %d %d %d\n", speed.front_left_speed, speed.front_right_speed, speed.rear_left_speed, speed.rear_right_speed);
			log_thrust(speed);
		}
		
		if(log_config.altitude_enabled) {
			altimeter_t altimeter_data;
			getFilteredAltitude(&altimeter_data);
			log_altimeter_data(altimeter_data);
		}
		
		last_log_ticks = current_ticks();
	}
	
#define STATUS_INTERVAL_MS 1000
//#define ENABLE_UART_STATUS
#ifdef ENABLE_UART_STATUS
	if(elapsed_time_ms(last_uart_ticks) > STATUS_INTERVAL_MS) {
		switch(current_state) {
			case(AWAIT_INIT):
			printf("Await Init\n");
			break;
			case(IDLE):
			printf("Idle\n");
			break;
			case(RUNNING):
			printf("Running\n");
			break;
			case(LANDING):
			printf("Landing\n");
			break;
			case(SHUTDOWN):
			printf("Shutdown\n");
			break;
			default:
			printf("No State\n");
		}
		printf("Thrust: %.2f\n", get_target_vertical_velocity());
		float target_x, target_y;
		get_target(&target_x, &target_y);
		printf("Target: %.2f %.2f\n", target_x, target_y);
		last_uart_ticks = current_ticks();
	}
#endif
}