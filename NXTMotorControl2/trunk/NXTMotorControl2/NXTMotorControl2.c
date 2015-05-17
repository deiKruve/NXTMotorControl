/*
 * NXTMotorControl2.c
 *
 * Created: 16.01.2015 19:43:35
 *	Author: Stefan Huber
 *	page	: http://stefanshacks.blogspot.de/
 *	License	: GNU General Public License
 *
 *	Update	: 16.05.2015
 */ 

//TODO implement acceleration

#define F_CPU	20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <util/atomic.h>
#include <stdlib.h>

#include "utils.h"
#include "twislave.h"
#include "systick.h"
#include "TachoControl.h"
#include "MotorControl.h"

/*
Global TODO
- determine direction and copy into i2c register
- finished PID regulation for speed and position
*/

//Device configuration
#define TWI_ADDR		0x03
#define DEVICE_ID		0x34

//Register map for i2c access 
/* 
txbuffer/rxbuffer identical, but ID could not be overwritten
Address	Width	Description
0		8		Sensor ID 0x34
1		8		Motor 0 Mode
2		8		Motor 0 Direction 
3		8		Motor 0 PWM 8 bit 
4-7		32		Motor 0 Current Pos(read) / set Pos (write)
8-11	32		Motor 0 Current Speed (read) / set Speed (write) (deltaTacho*SYSTICK_FREQ/deltaSystick) == tacho_count / s
12-13	16		Motor 0 Value P for PID control of speed / position
14-15	16		Motor 0 Value I for PID control of speed / position	
16-17	16		Motor 0 Value D for PID control of speed / position	 
 
*/
#define DEVICE_ID_OFFSET	0
#define MOTOR1_OFFSET		1
#define MOTOR2_OFFSET		MOTOR1_OFFSET + 18 

#define MOTOR_MODE			0
#define MOTOR_DIR			1
#define MOTOR_PWM			2
#define MOTOR_POS			(MOTOR_PWM +1)
#define MOTOR_SPEED			(MOTOR_POS +4) 
#define MOTOR_PID_P			(MOTOR_SPEED +4)
#define MOTOR_PID_I			(MOTOR_PID_P +2)
#define MOTOR_PID_D			(MOTOR_PID_I +2)

//Possible Motor modes
#define	MOTOR_MODE_FLOAT		0	//Motor free floating
#define MOTOR_MODE_BREAK		1	//Motor Break
#define	MOTOR_MODE_PWM			2	//Motor PWM Control
#define MOTOR_MODE_POS_CTRL		3	//Motor Position control with PID
#define MOTOR_MODE_SPEED_CTRL	4	//Motor Speed Control with PID


//Global variables
static int32_t tacho[2]			= {0, 0};	//Current tacho count
static int32_t tacho_prev[2]	= {0, 0};	//Tacho count of previous loop
static uint32_t tick_prev		=  0;		//previous value of systick

void motorStateMachine(uint8_t motor_chan, uint32_t deltaTick)
{
	//Offset for I2C interactions
	uint8_t offset = (motor_chan == 1) ? MOTOR1_OFFSET : MOTOR2_OFFSET;
	//Index for readout of data arrays
	uint8_t indx = motor_chan - 1;

	//calculate current speed (using deltaTick)
	int32_t deltaTacho = (tacho[indx] - tacho_prev[indx]);
	//int32_t speed = (deltaTacho * ((int32_t)(SYSTICK_FREQ*(int32_t)60UL)/(int32_t)TACHO_TICKS_PER_TURN))/(int32_t)deltaTick;
	int32_t speed = (deltaTacho * (int32_t)SYSTICK_FREQ)/(int32_t)deltaTick;
	tacho_prev[indx] = tacho[indx];
	writeDoubleWordToBuffer(speed, txbuffer, offset+MOTOR_SPEED);
		
	//read out target speed / target position for PID control
	int32_t setPos = getDoubleWordFromBuffer(rxbuffer, offset+MOTOR_POS);
	int32_t setSpeed = getDoubleWordFromBuffer(rxbuffer, offset+MOTOR_SPEED);

	//read out parameters for the PID selected control
	//TODO: optional: faster copy of the PID parameters from rxbuffer to txbuffer using memcpy
	int16_t P = getWordFromBuffer(rxbuffer, offset+MOTOR_PID_P);
	int16_t I = getWordFromBuffer(rxbuffer, offset+MOTOR_PID_I);
	int16_t D = getWordFromBuffer(rxbuffer, offset+MOTOR_PID_D);
	
	writeWordToBuffer(P, txbuffer, offset+MOTOR_PID_P);
	writeWordToBuffer(I, txbuffer, offset+MOTOR_PID_I);
	writeWordToBuffer(D, txbuffer, offset+MOTOR_PID_D);
	
	// state machine for motor
	
	//Variables necessary for motor control state machines
	uint8_t pwm = 0;
	uint8_t dir = 0;
	//helper variable to calculate PWM for position / speed PID
	int32_t pwm_raw = 0; 
	
	uint8_t mode = rxbuffer[offset+MOTOR_MODE];
	txbuffer[offset+MOTOR_MODE] = mode;
	switch(mode)
	{
		case MOTOR_MODE_FLOAT:
			pwm = 0;
			setMotorBreakMode(motor_chan, MOTOR_FLOAT);
			break;
		case MOTOR_MODE_BREAK:
			pwm = 0;
			setMotorBreakMode(motor_chan, MOTOR_MODE_BREAK);
			break;
		case MOTOR_MODE_PWM:
			pwm = rxbuffer[offset+MOTOR_PWM];
			dir = rxbuffer[offset+MOTOR_DIR];
		
			setMotorPWM(motor_chan, pwm );
			setMotorDir(motor_chan, dir );
			break;
		case MOTOR_MODE_POS_CTRL:
			pwm_raw = P * (tacho[indx] - setPos) + D * (speed);
			//TODO implement I part of PID
			if(pwm_raw > 0)
			{
				dir = MOTOR_BACKWARD;
			}
			else
			{
				dir = MOTOR_FORWARD;
			}
			pwm = (uint8_t)(abs(pwm_raw)); //Maximum speed can be set by MOTOR_PWM reg
			if(pwm >  rxbuffer[offset+MOTOR_PWM])
				pwm = rxbuffer[offset+MOTOR_PWM];
					
			setMotorPWM(motor_chan, pwm );
			setMotorDir(motor_chan, dir );
			break;
		case MOTOR_MODE_SPEED_CTRL:			
			dir = rxbuffer[offset+MOTOR_DIR];
			
			pwm_raw = getMotorPWM(motor_chan);
			pwm_raw += (abs(setSpeed) - abs(speed)) / P;
			pwm = (uint8_t)(abs(pwm_raw) & 0xff);
			
			setMotorPWM(motor_chan, pwm );
			setMotorDir(motor_chan, dir );
			
			break;
		
	}
	txbuffer[offset+MOTOR_PWM]	= pwm;
	txbuffer[offset+MOTOR_DIR]	= dir;

}


int main(void)
{
	//Initialize different peripherals
	cli();
	
	init_twi_slave(TWI_ADDR);
	init_systick();
	init_tacho();
	init_MotorGPIO();
	init_MotorPWM();
	
	//Clear both I2C buffers
	memset((void *)rxbuffer, 0, buffer_size);
	memset((void *)txbuffer, 0, buffer_size);
	//Write Device ID into the first i2c register
	txbuffer[DEVICE_ID_OFFSET] = DEVICE_ID;

	
	sei();
	
    while(1)
    {
		
		//Calculate time passed since previous loop
		
		uint32_t currentTick = getSystick();
		uint32_t deltaTick = currentTick - tick_prev;
		tick_prev = currentTick;
		deltaTick = (deltaTick == 0) ? 1 : deltaTick;//safety check to avoid div by zero

		//Read encoder and store current position into I2C register; not part of the motorStateMachine to ensure that timer readout and encode readout happen close to another
		tacho[0] += encode_read(1);
		tacho[1] += encode_read(2);
		writeDoubleWordToBuffer(tacho[0], txbuffer, MOTOR1_OFFSET + MOTOR_POS);
		writeDoubleWordToBuffer(tacho[1], txbuffer, MOTOR2_OFFSET + MOTOR_POS);
	
		motorStateMachine( 1 , deltaTick);
		motorStateMachine( 2 , deltaTick);
				
		_delay_ms(10);	      
    }
}