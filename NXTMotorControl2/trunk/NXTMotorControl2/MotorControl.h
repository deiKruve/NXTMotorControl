/*
 * MotorControl.h
 *
 * Created: 12.11.2014 18:02:46
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <inttypes.h>


#define MOTOR_FLOAT		0
#define MOTOR_BREAK		1
#define MOTOR_FORWARD	1
#define MOTOR_BACKWARD	0

void setMotorBreakMode(uint8_t chan, uint8_t mode);		//Sets the motor to either breaking or floating (MOTOR_BREAK or MOTOR_FLOAT)
void setMotorDir(uint8_t chan, uint8_t dir);			//Sets motor direction (either MOTOR_FORWARD or MOTOR_BACKWARD)
uint8_t getMotorDir(uint8_t chan);						//gets current motor direction
void init_MotorGPIO();									//Inits GPIOs for motor control
void init_MotorPWM();									//Inits Timer for motor PWM
void setMotorPWM(uint8_t chan,  uint8_t  pwm);			//Sets motor PWM value
uint8_t getMotorPWM(uint8_t chan);						//Gets the current motor pwm value

//GPIO Assignment
#define CTRL_DDR	DDRD
#define CTRL_PORT	PORTD
#define CTRL_2A		(1<<1)
#define CTRL_2B		(1<<2)
#define CTRL_1B		(1<<5)
#define CTRL_1A		(1<<6)

#define PWM_DDR		DDRB
#define PWM_PORT	PORTB
#define PWM1		(1<<1)
#define PWM2		(1<<2)

#define FAULT_DDR	DDRD
#define FAULT_PIN	PIND
#define FAULT12		(1<<7)

#define SLEEP_DDR	DDRB
#define SLEEP_PORT	PORTB
#define SLEEP_PIN	PINB
#define SLEEP12		(1<<0)

#define SPEED1		OCR1B	//OC1B
#define SPEED2		OCR1A	//OC1A



#endif /* MOTORCONTROL_H_ */