/*
 * MotorControl.c
 *
 * Created: 12.11.2014 18:04:39
 * 	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 * 
 *	Update	: 16.05.2015
 *
 */
#include "MotorControl.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

static uint8_t motor_dir[2]; //save current motor direction

#ifdef DEBUG
static uint8_t validateMotorChannel(uint8_t chan) //Validates motor channel. Always gives back a correct channel if invalid values given (only necessary for debug purposes)
{
	
	uint8_t result = chan;
	
	if(chan == 0)
	{
		return 1;
	}
	else if(chan > 2)
	{
		return 2;
	}
		
	return result;
}
#endif

uint8_t getMotorDir(uint8_t chan)
{
	#ifdef DEBUG
	chan = validateMotorChannel(chan);
	#endif
	return motor_dir[chan - 1];
}

void setMotorDir(uint8_t chan, uint8_t dir)
{
	#ifdef DEBUG
	chan = validateMotorChannel(chan);
	#endif
	
	if(motor_dir[chan - 1] != dir) //only change gpios if something has changes
	{
		motor_dir[chan - 1] = dir;
		
		if(chan == 1)
		{
			if(dir == MOTOR_FORWARD)
			{
				//CTRL_1A will be output-low
				CTRL_DDR |= CTRL_1A;
				CTRL_PORT &= ~CTRL_1A;
				//CTRL_1B will be input without pullup --> PWM applied here
				CTRL_DDR &= ~CTRL_1B;
				CTRL_PORT &= ~CTRL_1B;	
			}
			else
			{
				//CTRL_1B will be output-low
				CTRL_DDR |= CTRL_1B;
				CTRL_PORT &= ~CTRL_1B;
				//CTRL_1A will be input --> PWM applied here
				CTRL_DDR &= ~CTRL_1A;
				CTRL_PORT &= ~CTRL_1A;
			}
		}
		else if(chan == 2)
		{
			if(dir == MOTOR_FORWARD)
			{
				//CTRL_2A will be output-low
				CTRL_DDR |= CTRL_2A;
				CTRL_PORT &= ~CTRL_2A;
				//CTRL_2B will be input --> PWM applied here
				CTRL_DDR &= ~CTRL_2B;
				CTRL_PORT &= ~CTRL_2B;	
			}
			else
			{
				//CTRL_2B will be output-low
				CTRL_DDR |= CTRL_2B;
				CTRL_PORT &= ~CTRL_2B;
				//CTRL_2A will be input --> PWM applied here
				CTRL_DDR &= ~CTRL_2A;
				CTRL_PORT &= ~CTRL_2A;
			}
		}
	}
}

void init_MotorGPIO()
{
	//Set Fault input
	FAULT_DDR &= ~ FAULT12;
	//Set Sleep Pin output and high
	SLEEP_DDR |= SLEEP12;
	SLEEP_PORT |= SLEEP12;
	
	//Set CTRL_PORTs to low / no pullup
	CTRL_PORT &= ~(CTRL_1A | CTRL_2A | CTRL_1B | CTRL_2B);
	
	//Init GPIO for PWM output
	PWM_DDR |= (PWM1 | PWM2);
	

}

void init_MotorPWM()
{
	//Set Timer 1 for fast PWM mode
	
	TCCR1A |= ((1<<COM1A1) | (1<<COM1B1)); //non-inverting mode
	TCCR1A |= (1<<WGM10); //Fast PWM, 8 Bit mode
	TCCR1B |= (1<<WGM12); //Fast PWM, 8 bit mode
	TCNT1 = 0;
	
	TCCR1B |= ((1<<CS10) | (1<<CS11)); //Prescaler 64 --> ~1.2 kHz PWM Freq
	
}

void setMotorBreakMode(uint8_t chan, uint8_t mode)
{
	#ifdef DEBUG
	chan = validateMotorChannel(chan);
	#endif
	
	uint8_t tmp = 0;
	
	if(chan == 1)
	{
		 tmp = CTRL_1A | CTRL_1B;
	}
	else if(chan == 2)
	{
		 tmp = CTRL_2A | CTRL_2B;
		
	}
	motor_dir[chan - 1] = 0xff; //Change motor dir memory, so setMotorDir() recognizes stop / float and resets GPIOs
	setMotorPWM(chan, 0);
	CTRL_DDR |= tmp;
	switch(mode)
	{
		case MOTOR_FLOAT:
		setMotorPWM(chan, 0);
		CTRL_PORT &= ~tmp;
		break;
		case MOTOR_BREAK:
		setMotorPWM(chan, 0);
		CTRL_PORT |= tmp;
		break;
		
	}
	
}

uint8_t getMotorPWM(uint8_t chan)
{
	#ifdef DEBUG
	chan = validateMotorChannel(chan);
	#endif
	
	uint8_t speed = 0;
	if(chan == 1)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			speed = SPEED1;
		}
	}
	else if(chan == 2)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			speed = SPEED2;
		}
	}
	
	return speed;
}


void setMotorPWM(uint8_t chan, uint8_t  pwm)
{
	#ifdef DEBUG
	chan = validateMotorChannel(chan);
	#endif
	
	if(chan == 1)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			SPEED1 = pwm;
		}
	}
	else if(chan == 2)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			SPEED2 = pwm;
		}
	}
	
}