/*
 * utils.h
 *
 * Created: 07.03.2015 14:53:00
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015
 */ 


#ifndef UTILS_H_
#define UTILS_H_

#include <inttypes.h>

//util function to write 32-bit int to i2c buffer, ISR safe (buf has to be declared volatile to suppress warnings)
void writeDoubleWordToBuffer(int32_t data, volatile uint8_t * buf, uint8_t offset);

//Util function to read 32-bit int from i2c buffer, ISR safe
int32_t getDoubleWordFromBuffer(volatile uint8_t * buf, uint8_t offset);

//util function to write 16-bit int to i2c buffer, ISR safe (buf has to be declared volatile to suppress warnings)
void writeWordToBuffer(int16_t data, volatile uint8_t * buf, uint8_t offset);

//Util function to read 16-bit int from i2c buffer, ISR safe
int16_t getWordFromBuffer(volatile uint8_t * buf, uint8_t offset);





#endif /* UTILS_H_ */