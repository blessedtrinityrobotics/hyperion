/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 * THE OFFICIAL TEAM HYPERION ROBOT CODE 2018
 * This code was written by Carlos Saucedo & Ricardo Santamaria-Sarcos.
*/

package org.usfirst.frc.team4516.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;//Gyroscope


@SuppressWarnings("deprecation")//Yeah I don't give a damn but I need to have this code done
public class hecc extends SampleRobot{
	//Channels for motors/joysticks
	private static final int kFrontLeftChannel = 5;
	private static final int kRearLeftChannel = 1;
	private static final int kFrontRightChannel = 6;
	private static final int kRearRightChannel = 4;
	private static final int kLeftJoystickChannel = 0;
	private static final int kRightJoystickChannel = 1;
	private static final int kSlideMotor1 = 10;
	private static final int kSlideMotor2 = 9;
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
	
	//Sensor declarations
	private ADXRS450_Gyro onboardGyro;
	private Encoder e_frontRight;
	
	public void robotInit() {
		//Constructors for joysticks and motor controllers
		m_rightJoystick = new Joystick(kLeftJoystickChannel);
		m_leftJoystick = new Joystick(kRightJoystickChannel);
		m_frontLeft = new Spark(kFrontLeftChannel);
		m_frontRight = new Spark(kFrontRightChannel);
		m_rearLeft = new Spark(kRearLeftChannel);
		m_rearRight = new Spark(kRearRightChannel);
		m_slideMotor1 = new Spark(kSlideMotor1);
		m_slideMotor2 = new Spark(kSlideMotor2);
		
		//Constructor for RobotDrive
		//Note: X is left-right (strafe), Y is forward-backward, Z is rotation
		m_robotDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
		
		//Encoder
		e_frontRight = new Encoder(6, 7, false, Encoder.EncodingType.k1X);
		e_frontRight.setDistancePerPulse(e_distancePerPulse);
		
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
		e_frontRight.reset();
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
        	m_robotDrive.driveCartesian(m_rightJoystick.getX(), -m_rightJoystick.getY(), m_leftJoystick.getX(),0.0);
            //Timer.delay(0.02);
        }
    }
    
    //Functions for autonomous
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
}