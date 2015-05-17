/*
 * systick.c
 *
 * Created: 23.11.2014 17:18:35
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015
 */ 

#include "systick.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/atomic.h>

volatile static uint32_t tick;

uint32_t getSystick()
{
	uint32_t result;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		result = tick;
	}
	return result;
}

void init_systick()
{
		tick = 0;
		//Init Timer 0 for systick timer, CTC mode
		TCCR0A |= (1<<WGM01); //CTC Mode
		TCCR0B |= (1<<CS01) | (1<<CS00); //Prescaler 64
		OCR0A = SYSTICK_OCR; 
		TIMSK0 |= (1<<OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	tick++;

}


