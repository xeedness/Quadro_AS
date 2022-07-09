/*
 * util.c
 *
 * Created: 5/23/2021 12:58:53 PM
 *  Author: xeedn
 */ 


#include <asf.h>

#include "util.h"
#include "timer.h"


int util_get_clock_ms(unsigned long *count)
{
	*count = elapsed_time_ms(0);
	return 0;
}

int util_delay_ms(unsigned long num_ms)
{
	delay_ms(num_ms);
	return 0;
}

int util_log(const char* fmt, ...) {
	
}