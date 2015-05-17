package motorControl;
import lejos.nxt.*;
import lejos.util.*;

/** 
 * @author	: Stefan Huber
 * page		: http://stefanshacks.blogspot.de/
 * license	: GNU General Public License
 * @since	: 16.05.2015
 * 
 */
public class NXTMotor {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
				
		NXTMotorControl motor_ctrl = new NXTMotorControl(SensorPort.S1, NXTMotorControl.MotorTYPE_NXT, NXTMotorControl.MotorTYPE_NXT);
	
		int id = motor_ctrl.getBoardID();		
		LCD.drawString("ID: " + Integer.toString(id), 0, 0);
		
		
		int i = 0;
		while(Button.ESCAPE.isUp())
		{
			//Test for position control (rotateTo ...)
			/*
			 * motor_ctrl.M1.PIDPositionControl.PWMLimit = 100;
			if(Button.LEFT.isDown())
				i+=10;
			if(Button.RIGHT.isDown())
				i-=10;
			if(Button.ENTER.isDown())
				i=0;
			
			LCD.drawString("                   ", 0, 1);
			LCD.drawString("Set: " + Integer.toString(i), 0 ,1);
			
			LCD.drawString("                   ", 0, 2);
			LCD.drawString("Pos: " + Integer.toString(motor_ctrl.M1.getTachoCount()), 0, 2);
		
			
			motor_ctrl.M1.rotateTo(i, true);
			
			LCD.drawString("                   ", 0, 3);
			LCD.drawString("PWM " + Integer.toString(motor_ctrl.M1.getSpeedRAW()), 0, 3);
			
			Delay.msDelay(200);
			*/
			
			//Test for speed control
			motor_ctrl.M1.PIDSpeedControl.P = 10;
			if(Button.LEFT.isDown())
				i+=10;
			if(Button.RIGHT.isDown())
				i-=10;
			if(Button.ENTER.isDown())
				i=0;
			
			motor_ctrl.M1.setSpeed(i);
			
			if(i<0)
				motor_ctrl.M1.backward();
			else
				motor_ctrl.M1.forward();
			
			LCD.drawString("                   ", 0, 1);
			LCD.drawString("Set: " + Integer.toString(i) + " deg/s", 0 ,1);
			LCD.drawString("                   ", 0, 2);
			LCD.drawString("Speed: " + Integer.toString(motor_ctrl.M1.getSpeed())+" deg/s", 0, 2);
			LCD.drawString("                   ", 0, 3);
			LCD.drawString("PWM " + Integer.toString(motor_ctrl.M1.getSpeedRAW()), 0, 3);
			Delay.msDelay(100);
			
			motor_ctrl.M1.backward();
			
		}

		motor_ctrl.M1.flt();	
	}

}
