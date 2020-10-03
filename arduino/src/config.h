/*
 * IncFile1.h
 *
 * Created: 13.07.2020 00:17:46
 *  Author: xeedn
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

//#define STARTUP_TIME 5000
//#define RUN_TIME 10000
#define LANDING_TIME 10000

typedef struct sensor_config_s{
	float acceleration_weight;
	float measurement_error_angle;
	float measurement_error_angular_velocity;
	float estimate_error_angle;
	float estimate_error_angular_velocity;
	float altitude_gain;
	float speed_gain;
	uint8_t use_kalman_orientation;
	uint8_t enabled;
} sensor_config_t;

typedef struct pid_config_s{
	float pid_amplify_factor;
	float pid_angle_p_factor;
	float pid_angle_i_factor;
	float pid_angle_d_factor;
	float pid_rate_p_factor;
	float pid_rate_i_factor;
	float pid_rate_d_factor;
	float pid_vertical_velocity_p_factor;
	float pid_vertical_velocity_i_factor;
	float pid_vertical_velocity_d_factor;
	uint32_t update_interval_ms;
} pid_config_t;

typedef struct esc_config_s {
	float min_speed;
	float max_speed;
	float vertical_velocity_pid_amplifier;
	float vertical_velocity_limit;
	uint32_t update_interval_ms;
} esc_config_t;

typedef struct log_config_s {
	uint8_t orientation_enabled;
	uint8_t pid_enabled;
	uint8_t speed_enabled;	
	uint8_t altitude_enabled;
	uint32_t log_interval_ms;
} log_config_t;

extern pid_config_t pid_config;
extern esc_config_t esc_config;
extern log_config_t log_config;
extern sensor_config_t sensor_config;

#endif