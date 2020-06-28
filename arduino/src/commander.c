/*
 * commander.c
 *
 * Created: 27.06.2020 00:23:48
 *  Author: Philipp
 */ 

#include <inttypes.h>
#include "commander.h"
#include "log.h"

// DEBUG CODE ////////////////////////////////////////////////////////////
#include <asf.h>

#define LED (IOPORT_CREATE_PIN(PIOB, 27))

void init_led(void)
{
	static bool uglyInitFlag = false;
	if(uglyInitFlag) return;
	
	ioport_init();
	ioport_set_pin_dir(LED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_HIGH);
	uglyInitFlag = true;
}

void led_on(void)
{
	init_led();
	ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_HIGH);
}

void led_off(void)
{
	init_led();
	ioport_set_pin_level(LED, IOPORT_PIN_LEVEL_LOW);
}
//////////////////////////////////////////////////////////////////////////

bool commander_execute(uint8_t cmd)
{
	switch (cmd)
	{
		case 42:
		{
			log_enable();
			led_on();
			return true;
		}
		case 43:
		{
			log_disable();
			led_off();
			return true;
		}
		
		default:
		{
			return false;
		}
	}
}