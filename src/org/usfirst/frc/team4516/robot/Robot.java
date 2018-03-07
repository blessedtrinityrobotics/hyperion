/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 * THE OFFICIAL TEAM HYPERION ROBOT CODE 2018
 * This code was written by Carlos Saucedo & Zachary Moroski.
*/

package org.usfirst.frc.team4516.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;//Gyroscope
import edu.wpi.first.wpilibj.DoubleSolenoid;


@SuppressWarnings("deprecation")//Yeah I don't give a damn but I need to have this code done
public class Robot extends SampleRobot{
	//Channels for motors/joysticks
		private static final int kFrontLeftChannel = 5;
		private static final int kRearLeftChannel = 1;
		private static final int kFrontRightChannel = 6;
		private static final int kRearRightChannel = 4;
		private static final int kLeftJoystickChannel = 0;
		private static final int kRightJoystickChannel = 1;
		private static final int kFrontRightEncoderA = 6;
		private static final int kFrontRightEncoderB = 7;
		private static final int kRearRightEncoderA = 2;
		private static final int kRearRightEncoderB = 3;
		private static final int kFrontLeftEncoderA = 4;
		private static final int kFrontLeftEncoderB = 5;
		private static final int kRearLeftEncoderA = 0;
		private static final int kRearLeftEncoderB = 1;
		private static final int kSlideMotor1 = 2;
		private static final int kSlideMotor2 = 3;
		//sprivate static final int forwardArmSolenoidChan = 69;
		//sprivate static final int reverseArmSolenoidChan = 42;
		//sprivate static final int forwardClimbSolenoidChan = 500;
		//sprivate static final int reverseClimbSolenoidChan = 60;
		private double wheelCircumference = 25.132741228700002267; //Circumference
		private double e_distancePerPulse = wheelCircumference/1440 * 4; //Distance per pulse (circumference/pulses per revolution * 4)
		
		//MecanumDrive constructor
		private MecanumDrive m_robotDrive;

		//Controllers/etc
		private Timer m_timer = new Timer();
		private Joystick m_rightJoystick;
		private Joystick m_leftJoystick;
		
		//Drive Train Components
		private SpeedController m_frontLeft;
		private SpeedController m_frontRight;
		private SpeedController m_rearLeft;
		private SpeedController m_rearRight;
		private SpeedController m_slideMotor1;
		private SpeedController m_slideMotor2;
		
		//Solenoids
		//sprivate DoubleSolenoid armSolenoid;
		//sprivate DoubleSolenoid climbSolenoid;
		
		//Sensor declarations
		private ADXRS450_Gyro onboardGyro;
		private MecanumEncoder m_robotEncoder;
		private Encoder e_frontRight;
		private Encoder e_rearRight;
		private Encoder e_frontLeft;
		private Encoder e_rearLeft;
		private StopWatch m_clock;
		private double m_lastTime;
	
	public void robotInit() {
		//Constructors for joysticks and motor controllers
				m_rightJoystick = new Joystick(kRightJoystickChannel);
				m_leftJoystick = new Joystick(kLeftJoystickChannel);
				m_frontLeft = new Spark(kFrontLeftChannel);
				m_frontRight = new Spark(kFrontRightChannel);
				m_rearLeft = new Spark(kRearLeftChannel);
				m_rearRight = new Spark(kRearRightChannel);
				m_slideMotor1 = new Spark(kSlideMotor1);
				m_slideMotor2 = new Spark(kSlideMotor2);
				m_slideMotor2.setInverted(true);//Sets this one to be inverted
				
				//Solenoids
				//sarmSolenoid = new DoubleSolenoid(forwardArmSolenoidChan, reverseArmSolenoidChan);
				//sclimbSolenoid = new DoubleSolenoid(forwardClimbSolenoidChan, reverseClimbSolenoidChan);
				
				//Constructor for RobotDrive
				//Note: X is left-right (strafe), Y is forward-backward, Z is rotation
				m_robotDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
				
				//Encoders
				e_frontRight = new Encoder(kFrontRightEncoderA, kFrontRightEncoderB, false, Encoder.EncodingType.k1X);
				e_rearRight = new Encoder(kRearRightEncoderA, kRearRightEncoderB, false, Encoder.EncodingType.k1X);
				e_frontLeft = new Encoder(kFrontLeftEncoderA, kFrontLeftEncoderB, true, Encoder.EncodingType.k1X);
				e_rearLeft = new Encoder(kRearLeftEncoderA, kRearLeftEncoderB, true, Encoder.EncodingType.k1X);
				e_frontRight.setDistancePerPulse(e_distancePerPulse);
				e_rearRight.setDistancePerPulse(e_distancePerPulse);
				e_frontLeft.setDistancePerPulse(e_distancePerPulse);
				e_rearLeft.setDistancePerPulse(e_distancePerPulse);
				
				m_robotEncoder = new MecanumEncoder(e_frontRight, e_rearRight, e_frontLeft, e_rearLeft);
				
				//Reset Encoders
				m_robotEncoder.reset();
				
				
				//Clock
				m_clock = new StopWatch();
				m_clock.start();
				m_lastTime = 0.0;
				
				
				//Gyro
				 onboardGyro = new  ADXRS450_Gyro();
				 onboardGyro.calibrate();//Calibrates the gyro
				 onboardGyro.reset();//Sets gyro to 0 degrees
	}
	
	/**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() { 
		m_timer.reset();
		m_timer.start();
        while (isAutonomous() && isEnabled()) {
        	String gameData;
    		gameData = DriverStation.getInstance().getGameSpecificMessage();
    		if(gameData.charAt(1) == 'L')//First Switch is on left
    		{
    			
    		} else if(gameData.charAt(1) == 'R'){//First Switch is on right
    			
    		}
            //Timer.delay(0.02);
        }
    }

    /**
     * This function is called once each time the robot enters teleop mode.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	//Mecanum drive using 2 joysticks
        	m_robotDrive.driveCartesian(-m_rightJoystick.getX(), -m_rightJoystick.getY(), m_leftJoystick.getX(),0.0);
        	
        	//Slide rail controls
        	if(m_rightJoystick.getRawButton(3)) {
        		moveSlideUp(0.5);
        	}else if(m_rightJoystick.getRawButton(2)) {
        		moveSlideDown(0.25);
        	}else if(!m_rightJoystick.getRawButton(3) || !m_rightJoystick.getRawButton(2)) {
        		stopSlide();
        	}
        	
        	//Cube grabbing controls
        	if(m_leftJoystick.getTrigger()) {
        		grabCube();
        	}else if(m_leftJoystick.getRawButton(2)) {
        		releaseCube();
        	}else if(!m_leftJoystick.getTrigger() || m_leftJoystick.getRawButton(2)) {
        		stopCube();
        	}
            //Timer.delay(0.02);
        	
        	
        }
    }
    
    
    //Function
    public void moveForward(double distance) {
    	double initDistance = e_frontRight.getDistance();
    	while(e_frontRight.getDistance() < initDistance) {
    		m_robotDrive.driveCartesian(0.5, 0.0, 0.0, 0.0);
    	}
    	m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    }
    
    public void turnRight() {
    	double initBearing = onboardGyro.getAngle();
    	while(onboardGyro.getAngle() < initBearing + 90) {
    		m_robotDrive.driveCartesian(0.0, 0.0, 0.5, 0.0);
    	}
    	m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    }
    
    public void turnLeft() {
    	double initBearing = onboardGyro.getAngle();
    	while(onboardGyro.getAngle() > initBearing - 90) {
    		m_robotDrive.driveCartesian(0.0, 0.0, -0.5, 0.0);
    	}
    	m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    }
    
    //Slide rail controls
    public void moveSlideUp(double speed) {//moves the slide
 	   m_slideMotor1.set(speed);
 	   m_slideMotor2.set(-speed);
    }
    
    public void moveSlideDown(double speed) {
 	   m_slideMotor1.set(-speed);
 	   m_slideMotor2.set(speed);
    }
    
    public void stopSlide() {
 	   m_slideMotor1.set(0.0);
 	   m_slideMotor2.set(0.0);
 	   
    }
    
    //Pneumatic controls
    public void grabCube() {
    	
    }
    
    public void releaseCube() {
    	
    }
    
    public void stopCube() {
    	
    }
}