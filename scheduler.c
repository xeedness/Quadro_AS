/*
 * scheduler.c
 *
 * Created: 24.10.2018 21:07:56
 *  Author: Philipp
 */ 

#include "scheduler.h"
#include "delay.h"

#define PRESCALER 128
#define MS_2_CYCLES(ms) ((uint32_t)(ms*(F_CPU/1000)/PRESCALER))

void (*callback)(void) = NULL; 

void scheduler_init(size_t period_ms)
{	
	pmc_enable_periph_clk(ID_TC0);					// enable clock signal
	REG_TC0_CMR0	|= TC_CMR_TCCLKS_TIMER_CLOCK4	// prescale 128 (1.524 µs clk period)
					|	TC_CMR_CPCTRG;				// reset counter on match 
	REG_TC0_RC0		= MS_2_CYCLES(period_ms);		// set match value for given period
	REG_TC0_IER0	= TC_IER_CPCS;					// enable interrupt on compare match
	REG_TC0_IDR0	= ~TC_IDR_CPCS;					// disable other interrupts
	
	NVIC_EnableIRQ(TC0_IRQn);						// enable TC0 interrupts
}

void scheduler_setCB(void (*cb)(void))
{
	callback = cb;
}

void scheduler_clearCB(void)
{
	callback = NULL;
}

void TC0_Handler(void)
{
	uint32_t dummy = REG_TC0_SR0;	// has to be read to avoid continuous interrupts
	if(callback != NULL)
	{
		callback();
	}
}

void scheduler_start(void)
{
	REG_TC0_CCR0 = TC_CCR_SWTRG | TC_CCR_CLKEN;
}

void scheduler_stop(void)
{
	// NEVER GONNA STOP ME BITCH
}