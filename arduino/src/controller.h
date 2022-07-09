/*
 * controller.h
 *
 * Created: 27.10.2018 10:46:06
 *  Author: xeedn
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <asf.h>

enum InputConfig {
	ENABLE_ESP_POWER = 0x1,
	ENABLE_ESP_THROTTLE = 0x2,
	ENABLE_ESP_ROTATION = 0x4,
	ENABLE_IBUS_POWER = 0x8,
	ENABLE_IBUS_THROTTLE = 0x10,
	ENABLE_IBUS_ROTATION = 0x20,
	ENABLE_IBUS_FLIGHTMODE = 0x40
};

void controller_init(uint8_t input_config);
bool execute_command(uint8_t cmd, uint8_t* payload);
void relativeMax(float* dst, float* comparison);
void update_speed(void);
void interpret_channels(uint16_t* data, uint8_t channel_count);

uint8_t request_init(uint8_t is_repeat);
uint8_t request_init_status(void);

uint8_t is_controller_alive(void);

#endif /* CONTROLLER_H_ */