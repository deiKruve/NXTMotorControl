package motorControl;
import lejos.nxt.*;
import lejos.util.Delay;
import lejos.util.EndianTools;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.RegulatedMotorListener;

/**
 * @author	: Stefan Huber
 * page		: http://stefanshacks.blogspot.de/
 * license	: GNU General Public License
 * @since	: 16.05.2015
 * Update	: 16.05.2015
 * 
 * This class represents the NXTMotorControl board
 * 
 */
public class NXTMotorControl extends I2CSensor{
	private static final int I2C_ADDRESS = 0x03;
	/**
	 * Current board ID
	 */
	private static final byte BOARD_ID	= 0x34;
	
	/**
	 * Offset to address 0 for reading out board ID @see BOARD_ID
	 */
	private static final byte DEVICE_ID_OFFSET		= 0;
	/**
	 * Offset to address 0 for registers for Motor 1
	 */
	private static final byte MOTOR1_OFFSET			= 1;
	/**
	 * Offset to address 0 for registers for Motor 2
	 */
	private static final byte MOTOR2_OFFSET			= MOTOR1_OFFSET + 18;
	
	/**
	 * Reads a signed int from one of the boards i2c registers
	 * @param Offset within register map (MOTOR1_OFFSET or MOTOR2_OFFSET)
	 * @param register address of register
	 * @return result
	 */
	private int getInt(byte offset, byte register)
	{
		int result = 0;
		byte data[] = new byte[4];
		
		this.getData(offset+register, data, 4);
		result = EndianTools.decodeIntBE(data, 0);
				
		return result;
	}
	
	/**
	 * Write a signed int to one of the boards i2c registers
	 * @param value	value to write
	 * @param offset register offset (either MOTOR1_OFFSET or MOTOR2_OFFSET)
	 * @param register actual register to write
	 */
	private void setInt(int value, byte offset, byte register)
	{
		byte data[] = new byte[4];
		EndianTools.encodeIntBE(value, data, 0);
		this.sendData(offset+register, data, 4);
	}
	
	private short getShort(byte offset, byte register)
	{
		short result = 0;
		byte data[] = new byte[4];
				
		this.getData(offset+register, data, 4);
		result = EndianTools.decodeShortBE(data, 0);
				
		return result;
	}
	
	private void setShort(short value, byte offset, byte register)
	{
		byte data[] = new byte[4];
		EndianTools.encodeShortBE(value, data, 0);
		this.sendData(offset+register, data, 2);
	}
	
	/**
	 * 
	 * @author Stefan Huber
	 * This class represents the motors actually controlled by the board
	 */
	
	class ControlledMotor implements RegulatedMotor 
	{
		//I2C Registers for motor control
		private static final byte MOTOR_MODE			= 0;
		private static final byte MOTOR_DIR				= 1;
		private static final byte MOTOR_PWM				= 2;
		private static final byte MOTOR_POS				= 3;
		private static final byte MOTOR_SPEED			= MOTOR_POS+4;
		private static final byte MOTOR_PID_P			= MOTOR_SPEED +4;
		private static final byte MOTOR_PID_I			= MOTOR_PID_P +2;
		private static final byte MOTOR_PID_D			= MOTOR_PID_I +2;
		
		//Possible motor modes
		private static final byte MOTOR_MODE_FLOAT 		= 0;
		private static final byte MOTOR_MODE_BREAK		= 1;
		private static final byte MOTOR_MODE_PWM		= 2;
		private static final byte MOTOR_MODE_POS_CTRL	= 3;
		private static final byte MOTOR_MODE_SPEED_CTRL	= 4;
		
		private static final byte FORWARD			= 1;
		private static final byte BACKWARD			= 0;
		
		/**
		 * Register offset for this motor (either @see MOTOR1_OFFSET or @see MOTOR2_OFFSET)
		 */
		private byte offset;
		/**
		 * Maximum speed this motor may achieve; set by constructor
		 */
		private int MaxRPM;
		/** 
		 * link to the NXTMotorControl object containing this motor
		 */
		private NXTMotorControl ctrl;
		/**
		 * Offset between tacho count actually measured and read out --> allows to locally use @see resetTachoCount()
		 */
		private int TachoOffset; //Allows to set resetTachoCount - but only locally
		
		private void setMotorMode(byte mode)
		{
			ctrl.sendData(offset+MOTOR_MODE, mode);
		}
		
		/**
		 * Return RAW Pwm value with sign according to set direction
		 * @return
		 */
		private int getRawPWM()
		{
			int result = 0;
			byte data[] = new byte[1];
			
			ctrl.getData(offset+MOTOR_PWM, data, 1);
			result = data[0];
			
			//Correction because byte is implemented as signed. Convert it to unsigned 
			if(result < 0)
			{
				result = (256+result);
			}
			
			ctrl.getData(offset+MOTOR_DIR, data, 1);
			if(data[0] == BACKWARD)
			{
				result *= -1;
			}
			
			return result;
		}
		
		/**
		 * Sets raw pwm value. Motor direction determined by sign (+ forward, -backward)
		 * @param rawSpeed pwm value and direction (set by sign)
		 */
		private void setRawPWM(int  rawSpeed)
		{	
			if(rawSpeed < 0)
			{
				ctrl.sendData(offset+MOTOR_DIR, BACKWARD);
			}
			else
			{
				ctrl.sendData(offset+MOTOR_DIR, FORWARD);
			}
			int pwm = Math.abs(rawSpeed) % 255;
			
			
			ctrl.sendData(offset+MOTOR_PWM, (byte) pwm);
				
		}	
		
		/**
		 * Sends all the three parameters for P, I, D to the controler
		 * @param P
		 * @param I
		 * @param D
		 */
		private void setPID(int P, int I, int D)
		{
			ctrl.setShort((short) P, offset, MOTOR_PID_P);
			ctrl.setShort((short) I, offset, MOTOR_PID_I);
			ctrl.setShort((short) D, offset, MOTOR_PID_D);
		}
		
		/**
		 * Sets motor speed using degree / second
		 * @param speed
		 */
		private void setMotorSpeed(int speed)
		{
			int maxdegpersecond = (MaxRPM * 360) / 60;
			int rawpwm = (255*speed)/maxdegpersecond;
			setRawPWM(rawpwm);
			
		}
		
		/** 
		 * Gets the current motor speed in degree / s
		 * @return
		 */
		private int getMotorSpeed()
		{
			int result = 0;
			
			result = getInt(offset, MOTOR_SPEED); //gives speed in tacho ticks per second (720 ticks per turn)
			result = result / 2; // div by 2 to convert 720 ticks/turn to 360 degree/turn
			
			return result;
		}
		
		/**
		 * Gets raw and non-corrected tacho count of the motor - its position since start
		 * @return
		 */
		private int getMotorPosition()
		{
			int result = 0;
			result = getInt(offset, MOTOR_POS);
			return result;
		}
		
		class PIDParameter 
		{
			public short P;
			public short I;
			public short D;
			/**
			 * PWMLimit is only neccary for position control. It contains the maximum pwm value and therefore the maximum speed possible during rotation to a new position
			 */
			public int PWMLimit;
			
			public PIDParameter(short p, short i, short d, int pwm_limit )
			{
				P = p;
				I = i;
				D = d;
				PWMLimit = pwm_limit;
			}
		}
		
		/**
		 * Sends a set of PID parameters to the board
		 * @param value
		 */
		
		private void sendPIDValues(PIDParameter value)
		{
			this.setPID(value.P, value.I, value.D);
			int pwm = Math.abs(value.PWMLimit) % 255;
			ctrl.sendData(offset+MOTOR_PWM, (byte) pwm);
		}
		
		/**
		 * Store the PID control parameters for position control
		 */
		public PIDParameter PIDPositionControl;
		/**
		 * Store the PID control parameters for speed control
		 */
		public PIDParameter PIDSpeedControl;
		

				
		public ControlledMotor(NXTMotorControl control, byte RegisterOffset, int MotorType)
		{
			ctrl = control;
			offset = RegisterOffset;
			MaxRPM = MotorType;
			TachoOffset = 0;
			PIDPositionControl = new PIDParameter((short)20, (short)0 , (short)0 , 255);
			PIDSpeedControl = new PIDParameter((short)8, (short)0, (short)0, 0);
		}

		@Override
		public void forward() {
			sendPIDValues(this.PIDSpeedControl);
			this.setMotorMode(ControlledMotor.MOTOR_MODE_SPEED_CTRL);
			ctrl.sendData(offset+MOTOR_DIR, FORWARD);
		}

		@Override
		public void backward() {
			sendPIDValues(this.PIDSpeedControl);
			this.setMotorMode(ControlledMotor.MOTOR_MODE_SPEED_CTRL);
			ctrl.sendData(offset+MOTOR_DIR, BACKWARD);
		}

		@Override
		public void stop() {
			stop(false);			
		}

		@Override
		public void flt() {
			flt(false);
		}

		@Override
		public boolean isMoving() {
			if(getMotorSpeed() != 0)
			{
				return true;
			}
			return false;
		}

		@Override
		public int getRotationSpeed() {
			return getMotorSpeed();
		}

		@Override
		public int getTachoCount() {
			return (getMotorPosition() - TachoOffset)/2; //Divide by 2 because leJOS uses 360 ticks per turn (==degree), NXTMotorControl 720 Ticks per turn
		}
		
		/**
		 * @return gives the raw tacho count as provided by the NXTMotorControl board 
		 */ 
		public int getRawTachoCount() {
			return getMotorPosition();

		}

		@Override
		public void resetTachoCount() {
			TachoOffset = getMotorPosition();	
		}

		@Override
		public void addListener(RegulatedMotorListener listener) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public RegulatedMotorListener removeListener() {
			// TODO Auto-generated method stub
			return null;
		}

		@Override
		public void stop(boolean immediateReturn) {
			//TODO implement check of state when immediateReturn == false
			setMotorMode(MOTOR_MODE_BREAK);
		}

		@Override
		public void flt(boolean immediateReturn) {
			//TODO implement check of state when immediateReturn == false
			setMotorMode(MOTOR_MODE_FLOAT);
			
		}

		@Override
		public void waitComplete() {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void rotate(int angle, boolean immediateReturn) {
			this.rotateTo((angle) + this.getTachoCount(), immediateReturn);
			
		}

		@Override
		public void rotate(int angle) {
			rotate(angle, false);
			
		}

		@Override
		public void rotateTo(int limitAngle) {
			rotateTo(limitAngle, false);
		}

		
		@Override
		public void rotateTo(int limitAngle, boolean immediateReturn) {	
			sendPIDValues(this.PIDPositionControl);
			//this.setPID(20, 0, 0);
			ctrl.setInt((limitAngle*2)+this.TachoOffset, this.offset, ControlledMotor.MOTOR_POS);
			this.setMotorMode(MOTOR_MODE_POS_CTRL);
			
			if(immediateReturn == false)
			{
				//Wait until position is set within 2 degrees
				while( Math.abs(this.getTachoCount() - limitAngle) < 3)
				{
					Delay.msDelay(20);
				}
				
			}
			
		}

		@Override
		public int getLimitAngle() {
			return 0;
		}

		@Override
		public void setSpeed(int speed) {
			ctrl.setInt(speed*2, this.offset, ControlledMotor.MOTOR_SPEED);
		}
				
		/** 
		 * Sets the raw pwm value (-255 - 255). Sign indicates direction
		 * @param pwm value -255 to 255
		 */
		public void setSpeedRAW(int speed)
		{
			setRawPWM(speed);
		}
		
		/**
		 * Gets the raw pwm value. Sign indicated direction
		 * @return pwm value -255 to 255
		 */
		public int getSpeedRAW()
		{
			return this.getRawPWM();
		}

		@Override
		public int getSpeed() {
			return getMotorSpeed();
		}

		@Override
		public float getMaxSpeed() {
			return (MaxRPM * 360) / 60;
		}

		@Override
		public boolean isStalled() {
			// TODO Auto-generated method stub
			return false;
		}

		@Override
		public void setStallThreshold(int error, int time) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void setAcceleration(int acceleration) {
			// TODO Auto-generated method stub
			
		}
	}
	
	/**
	 * Motor type used for old Lego Mindstorms NXT Motors
	 */
	public static final int MotorTYPE_NXT = 170;
	/**
	 * Motor type used for new Lego Mindstorms Large EV3 Motors
	 */
	public static final int MotorTYPE_EV3L = 175;
	/**
	 * Motor type used for new Lego Mindstorms Medium EV3 Motors
	 */
	public static final int MotorTYPE_EV3M = 260;
		
	/**
	 * Regulated Motor attached to motor port 1
	 */
	public ControlledMotor M1;
	/**
	 * Regulated Motor attached to motor port 2
	 */
	public ControlledMotor M2;

	public NXTMotorControl(I2CPort port, int MotorType1, int MotorType2) {
		super(port, I2C_ADDRESS);
		M1 = new ControlledMotor(this, MOTOR1_OFFSET, MotorType1);
		M2 = new ControlledMotor(this, MOTOR2_OFFSET, MotorType2);
	}
	
	/**
	 * Returns board ID. Should be BOARD_ID for current NXTMotorControl2 firmware
	 * @return
	 */
	public byte getBoardID()
	{
		byte data[] = new byte[1];
		
		this.getData(DEVICE_ID_OFFSET, data, 1);
		
		return data[0];
	}
	
	/**
	 * Checks wheter the board ID has the expected value 
	 * @return true if ID is correct
	 */
	public boolean checkBoardID()
	{
		return getBoardID()==BOARD_ID;
	}
	

}
