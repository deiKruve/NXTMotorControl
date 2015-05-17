/*
 * TachoControl.c
 *
 * Created: 12.11.2014 18:07:53
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015 
 */

#include "TachoControl.h" 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/atomic.h>

//volatile static uint32_t tick;

volatile static int8_t tacho1_delta;
static int8_t tacho1_last;
volatile static int8_t tacho2_delta;
static int8_t tacho2_last;


void init_tacho()
{
	//Init GPIO for Tacho Encoder input
	TACHO1_DDR &= ~(TACHO1A | TACHO1B);
	TACHO2_DDR &= ~(TACHO2A | TACHO2B);
	

	//Init initial values for tacho 1
	int8_t new = 0;
	if(TACHO1_PHASEA)
	{
		new = 3;
	}
	if(TACHO1_PHASEB)
	{
		new ^= 1;
	}
	tacho1_last = new;
	tacho1_delta = 0;
	
	//Init initial values for tacho 2
	new = 0;
	if(TACHO2_PHASEA)
	{
		new = 3;
	}
	if(TACHO2_PHASEB)
	{
		new ^= 1;
	}
	tacho2_last = new;
	tacho2_delta = 0;

	//Pin change interrupts
	PCMSK1 |= TACHO1_ISR_MSK | TACHO2_ISR_MSK;
	PCICR |= (1<<PCIE1);
	
}

ISR(PCINT1_vect)
{

	int8_t new_state;
	int8_t diff;
	

	new_state = 0;
	if(TACHO1_PHASEA)
	{
		new_state = 3;
	}
	if(TACHO1_PHASEB)
	{
		new_state ^= 1;
	}
	diff = tacho1_last - new_state;
	if(diff & 1)
	{
		tacho1_last = new_state;
		tacho1_delta += (diff & 2) - 1;
	}

	new_state = 0;
	if(TACHO2_PHASEA)
	{
		new_state = 3;
	}
	if(TACHO2_PHASEB)
	{
		new_state ^= 1;
	}
	diff = tacho2_last - new_state;
	if(diff & 1)
	{
		tacho2_last = new_state;
		tacho2_delta += (diff & 2) - 1;
	}

}


int8_t encode_read(uint8_t chan)
{
	int8_t val = 0;
	if(chan == 1)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			val = tacho1_delta;
			tacho1_delta = 0;
			
		}
	}
	else 
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			val = tacho2_delta;
			tacho2_delta = 0;
		}
	}
	return val;
}
