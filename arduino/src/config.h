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

typedef struct pid_config_s{
	float pid_factor;
	float pid_p_factor;
	float pid_i_factor;
	float pid_d_factor;
	uint32_t update_interval_ms;
} pid_config_t;

typedef struct esc_config_s {
	uint16_t landing_speed;
	uint16_t hover_speed;
	uint16_t max_speed;
	uint16_t min_speed;
	uint32_t update_interval_ms;
} esc_config_t;

typedef struct log_config_s {
	uint8_t orientation_enabled;
	uint8_t pid_enabled;
	uint8_t speed_enabled;	
	uint32_t log_interval_ms;
} log_config_t;

extern pid_config_t pid_config;
extern esc_config_t esc_config;
extern log_config_t log_config;

#endif