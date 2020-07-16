/*
 * status_display.c
 *
 * Created: 14.07.2020 13:34:07
 *  Author: xeedn
 */ 

#include <asf.h>
#include <stdbool.h>
#include "status_display.h"


#define LED (IOPORT_CREATE_PIN(PIOB, 27))

void status_display_init(void)
{
	static bool uglyInitFlag = false;
	if(uglyInitFlag) return;
	
	ioport_init();
	ioport_set_pin_dir(LED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_HIGH);
	uglyInitFlag = true;
}

void set_init_led(uint8_t on)
{
	status_display_init();
	ioport_set_pin_level(LED, on ? IOPORT_PIN_LEVEL_HIGH : IOPORT_PIN_LEVEL_LOW);
}
