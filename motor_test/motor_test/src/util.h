/*
 * util.h
 *
 * Created: 5/23/2021 12:58:43 PM
 *  Author: xeedn
 */ 


#ifndef UTIL_H_
#define UTIL_H_

#include <stdarg.h>

int util_get_clock_ms(unsigned long *count);
int util_delay_ms(unsigned long num_ms);
int util_log(const char* fmt, ...);


#endif /* UTIL_H_ */