/*
 * utils.c
 *
 * Created: 07.03.2015 14:52:47
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015
 */ 

#include "utils.h"

#include <avr/interrupt.h>
#include <util/atomic.h>

//Help function to write 32-bit int to i2c buffer, ISR safe (buf has to be declared volatile to suppress warnings)
void writeDoubleWordToBuffer(int32_t data, volatile uint8_t * buf, uint8_t offset)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		buf[offset+0] = (uint8_t)((data >> 24)  & 0xff);
		buf[offset+1] = (uint8_t)((data >> 16)  & 0xff);
		buf[offset+2] = (uint8_t)((data >> 8 )  & 0xff);
		buf[offset+3] = (uint8_t)((data >> 0 )  & 0xff);
	}
}

int32_t getDoubleWordFromBuffer(volatile uint8_t * buf, uint8_t offset)
{
	int32_t result = 0;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		result |= (((int32_t)buf[offset+0]) << 24);
		result |= (((int32_t)buf[offset+1]) << 16);
		result |= (((int32_t)buf[offset+2]) <<  8);
		result |= (((int32_t)buf[offset+3]) <<  0);
	}
	
	return result;
}

void writeWordToBuffer(int16_t data, volatile uint8_t * buf, uint8_t offset)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		buf[offset+0] = (uint8_t)((data >> 8)  & 0xff);
		buf[offset+1] = (uint8_t)((data >> 0)  & 0xff);
	}
}

int16_t getWordFromBuffer(volatile uint8_t * buf, uint8_t offset)
{
	int16_t result = 0;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		result |= (((int16_t)buf[offset+0]) <<  8);
		result |= (((int16_t)buf[offset+1]) <<  0);
	}
	
	return result;
}

