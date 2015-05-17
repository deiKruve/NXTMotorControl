/*
 * TachoControl.h
 *
 * Created: 12.11.2014 18:07:41
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015
 */ 

#ifndef TACHOCONTROL_H_
#define TACHOCONTROL_H_

#include <inttypes.h>

//Configuration of Tacho Readout frequency


#define TACHO_TICKS_PER_TURN	720UL


//GPIO Assignment
#define TACHO1_DDR		DDRC
#define TACHO1_PORT		PORTC
#define TACHO1_PIN		PINC
#define TACHO1B			(1<<PC0)
#define TACHO1A			(1<<PC1)
#define TACHO1_PHASEB	(TACHO1_PIN & TACHO1A)
#define TACHO1_PHASEA	(TACHO1_PIN & TACHO1B)
#define TACHO1_ISR_MSK	((1<<PCINT9) | (1<<PCINT8))

#define TACHO2_DDR		DDRD
#define TACHO2_PORT		PORTD
#define TACHO2_PIN		PIND
#define TACHO2A			(1<<PD4)
#define TACHO2B			(1<<PD3)
#define TACHO2_PHASEA	(TACHO2_PIN & TACHO2A)
#define TACHO2_PHASEB	(TACHO2_PIN & TACHO2B)
#define TACHO2_ISR_MSK	((1<<PCINT11) | (1<<PCINT12))

void init_tacho();
int8_t encode_read(uint8_t chan);




#endif /* TACHOCONTROL_H_ */