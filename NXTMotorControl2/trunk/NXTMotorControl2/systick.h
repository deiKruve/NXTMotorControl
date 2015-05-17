/*
 * systick.h
 *
 * Created: 23.11.2014 17:17:05
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015
 */ 


#ifndef SYSTICK_H_
#define SYSTICK_H_
#include <inttypes.h>

#ifndef F_CPU
#error "F_CPU nicht definiert"
#endif


#define SYSTICK_FREQ			1000UL 
#define SYSTICK_OCR				( (F_CPU/(2UL*64UL*SYSTICK_FREQ)) -1UL)


uint32_t getSystick();
void init_systick();



#endif /* SYSTICK_H_ */