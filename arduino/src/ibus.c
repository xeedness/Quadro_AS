/*
 * ibus.c
 *
 * Created: 6/7/2021 10:04:46 PM
 *  Author: xeedn
 */ 
#include "ibus.h"

typedef struct ibus_state {
	uint8_t state;
	uint16_t checksum;
	uint8_t datal;
	uint8_t channel_count;
} ibus_state_t;


ibus_state_t state;
uint16_t data[IBUS_CHANNEL_COUNT];

void ibus_setup(void) {
	state.state = 0;
	state.channel_count = IBUS_CHANNEL_COUNT;
}

int ibus_read(uint8_t ch) {
	switch (state.state) {
		case 0:
		if (ch == 0x20) {
			state.checksum = 0xFFFF - 0x20;
			state.state = 1;
		}
		break;
		case 1:
		if (ch == 0x40) {
			state.state = 2;
			state.checksum -= ch;
		} else {
			state.state = 0;
		}
		break;
		case 30:
		state.datal = ch;
		state.state = 31;
		break;
		case 31: {
			uint_fast16_t checksum = (ch << 8) | state.datal;
			state.state = 0;
			if (checksum == state.checksum)
				return 0;
		} break;
		default:
		// Ignore these bytes if we've filled all of the channels
		if (state.state / 2 <= state.channel_count) {
			if ((state.state & 1) == 0) {
				// Data low byte
				state.datal = ch;
				} else {
				// Data high byte
				data[(state.state / 2) - 1] = (ch << 8) | state.datal;
			}
		}
		state.checksum -= ch;
		++state.state;
		break;
	}

	return -1;
}

uint16_t* ibus_data(void) {
	return data;	
}