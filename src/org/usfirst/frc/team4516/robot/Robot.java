/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 * THE OFFICIAL TEAM HYPERION ROBOT CODE 2018
 * This code was written by Carlos Saucedo & Zachary Moroski.
 * This robot utilizes Mecanum wheels.
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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;


@SuppressWarnings("deprecation")
public class Robot extends SampleRobot{
        /*
         * Variables for autonomous cases. 
         * initPosition values: L, M, R (fairly straightforward)
         * initGoal values: c, w (c -> scale; w -> switch)
         */
        char initPosition = 'L';
        char initGoal = 'w';
        
	    //Channels for motors/joysticks
		private static final int kFrontLeftChannel = 5;
		private static final int kRearLeftChannel = 1;
		private static final int kFrontRightChannel = 6;
		private static final int kRearRightChannel = 4;
		private static final int kLeftJoystickChannel = 0;
		private static final int kRightJoystickChannel = 1;
		private static final int kDarioJoystickChannel = 2;
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
		private static final int forwardArmSolenoidChan = 2;
		private static final int reverseArmSolenoidChan = 3;
		private double wheelCircumference = 25.132741228700002267; //Circumference (in inches)
		private double e_distancePerPulse = wheelCircumference/1440 * 4 * 1.35068534; //Distance per pulse (circumference/pulses per revolution * 4)
		private double slideMovementScaleTime = 7.0; //TO-DO
		private double slideMovementSwitchTime = 3.5; //TO-DO
		
		//MecanumDrive constructor
		private MecanumDrive m_robotDrive;

		//Controllers/etc
		private Timer m_timer = new Timer();
		private Joystick m_rightJoystick;
		private Joystick m_leftJoystick;
		private Joystick m_darioJoystick;
		
		//Drive Train Components
		private SpeedController m_frontLeft;
		private SpeedController m_frontRight;
		private SpeedController m_rearLeft;
		private SpeedController m_rearRight;
		private SpeedController m_slideMotor1;
		private SpeedController m_slideMotor2;
		
		//Solenoids
		private DoubleSolenoid armSolenoid;
		
		//Sensor declarations
		private ADXRS450_Gyro onboardGyro;
		private MecanumEncoder m_robotEncoder;
		private Encoder e_frontRight;
		private Encoder e_rearRight;
		private Encoder e_frontLeft;
		private Encoder e_rearLeft;
		
		
	
	public void robotInit() {
		//Constructors for joysticks and motor controllers
				m_rightJoystick = new Joystick(kRightJoystickChannel);
				m_leftJoystick = new Joystick(kLeftJoystickChannel);
				m_darioJoystick = new Joystick(kDarioJoystickChannel);
				m_frontLeft = new Spark(kFrontLeftChannel);
				m_frontRight = new Spark(kFrontRightChannel);
				m_rearLeft = new Spark(kRearLeftChannel);
				m_rearRight = new Spark(kRearRightChannel);
				m_slideMotor1 = new Spark(kSlideMotor1);
				m_slideMotor2 = new Spark(kSlideMotor2);
				m_slideMotor2.setInverted(true);
				
				
				//Solenoids
				armSolenoid = new DoubleSolenoid(forwardArmSolenoidChan, reverseArmSolenoidChan);
				
				//Constructor for RobotDrive
				//Note: X is forward-backward, Y is left-right(strafe), Z is rotation
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
				
				//Gyro
				 onboardGyro = new  ADXRS450_Gyro();
				 onboardGyro.calibrate();//Calibrates the gyro
				 onboardGyro.reset();//Sets gyro to 0 degrees
				 
				 /*
				  * Camera: try catch statement so that code does not crash if camera does not work.
				  */
				 try{
				     UsbCamera camera = new UsbCamera("cam0", 0);
				     camera.setFPS(30); //30 FPS
				     camera.setResolution(320, 240); //320 = width, 240 = height
				     CameraServer.getInstance().startAutomaticCapture(camera);
					 //CameraServer camera = CameraServer.getInstance();
					 //camera.startAutomaticCapture();
				 }catch(ArrayIndexOutOfBoundsException e){
				        System.out.println("Camera failed! Continuing program");
					 }
			}
	
	/**
	 * 
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() { 
		m_timer.reset();
		m_timer.start();
        	String gameData;
    		gameData = DriverStation.getInstance().getGameSpecificMessage();
    		System.out.println("Field Data: " + gameData);
    		moveForward(64);
    		switch(initPosition) {
            case 'L':
                switch(initGoal){
                    case 'c':
                        if(gameData.charAt(2) == 'L') {//First scale is on left
                            moveForward(340.0);
                            turnRight();
                            moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(10.0);
                            releaseCube(true);
                            moveBackward(18.0);
                            
                        } else if(gameData.charAt(2) == 'R') {//First scale is on right
                            moveForward(240.0);
                            turnRight();
                            moveForward(230.0);
                            turnLeft();
                            moveForward(80.0);
                            turnLeft();
                            moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(10.0);
                            releaseCube(true);
                            moveBackward(18.0);
                    }   
                    break;
                    
                    case 'w':
                        if(gameData.charAt(1) == 'L') {//First switch is on left
                            moveForward(113.0);
                            releaseCube(true);
                        
                        } else if(gameData.charAt(1) == 'R') {//First switch is on right
                            moveForward(24.0);
                            turnRight();
                            moveForward(86.0);
                            turnLeft();
                            moveForward(82.0);
                            releaseCube(true);
                    }
                    break;
                }
            break;
            
            case 'M':
                switch(initGoal){
                    case 'c':
                        if(gameData.charAt(2) == 'L') {//First scale is on left
    			            moveForward(60.0);
    			            turnLeft();
    			            moveForward(140.0);
    			            turnRight();
    			            moveForward(260.0);
    			            turnRight();
    			            moveSlideUp(0.5, slideMovementScaleTime);
    			            moveForward(10.0);
    			            releaseCube(true);
    			            moveBackward(18.0);
    			
    		          } else if(gameData.charAt(2) == 'R') {//First scale is on right
    			            moveForward(60.0);
    			            turnRight();
    			            moveForward(80.0);
    			            turnLeft();
    			            moveForward(260.0);
    			            turnLeft();
    			            moveSlideUp(0.5, slideMovementScaleTime);
    			            moveForward(10.0);
    			            releaseCube(true);
    			            moveBackward(18.0);
    		        }        
                    break;
                    
                    case 'w':
                        if(gameData.charAt(1) == 'L') {//First switch is on left
                            moveForward(24.0);
                            turnLeft();
                            moveForward(26.0);
                            turnRight();
                            moveForward(82.0);
                            releaseCube(true);
                    } else if(gameData.charAt(1) == 'R') {//First switch is on right
                           moveForward(24.0);
                           turnRight();
                           moveForward(26.0);
                           turnLeft();
                           moveForward(82.0);
                           releaseCube(true);
                    }        
                    break;
                }
    		break;
            
            case 'R':
                switch(initGoal){
                    case 'c':
                        if(gameData.charAt(2) == 'L') {//First scale is on left
                            moveForward(240.0);
                            turnLeft();
                            moveForward(230.0);
                            turnRight();
                            moveForward(80.0);
                            turnRight();
                            moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(10.0);
                            releaseCube(true);
                            moveBackward(18.0);
                            
                        } else if(gameData.charAt(2) == 'R') {//First scale is on right
                            moveForward(340.0);
                            turnLeft();
                            moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(10.0);
                            releaseCube(true);
                            moveBackward(18.0);
                    }   
                    break;
                    
                    case 'w':
                        if(gameData.charAt(1) == 'L') {//First switch is on left
                            moveForward(24.0);
                            turnLeft();
                            moveForward(86.0);
                            turnRight();
                            moveForward(82.0);
                            releaseCube(true);
                        
                        } else if(gameData.charAt(1) == 'R') {//First switch is on right
                            moveForward(106.0);
                            releaseCube(true);
                    }
                    break;
                }
            break;
        }
    }
    /**
     * This function is called once each time the robot enters teleop mode.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	//Mecanum drive using 2 joysticks
        	m_robotDrive.driveCartesian(m_rightJoystick.getX(), -m_rightJoystick.getY(), m_leftJoystick.getX(),0.0);
        	
        	//Slide rail controls
        	if(m_darioJoystick.getRawButton(3)) {
        		moveSlideUp(0.75);
        	}else if(m_darioJoystick.getRawButton(2)) {
        		moveSlideDown(0.5);
        	}else if(!m_darioJoystick.getRawButton(3) || !m_darioJoystick.getRawButton(2)) {
        		stopSlide(false);
        	}
        	
        	//Cube grabbing controls
        	if(m_darioJoystick.getTrigger()) {
        		grabCube(false);
        	}else if(m_darioJoystick.getRawButton(4)) {
        		releaseCube(false);
        	}else if(!m_darioJoystick.getTrigger() || !m_darioJoystick.getRawButton(4)) {
        		stopCube(false);
        	}
            Timer.delay(0.02);
        }
    }
    
    public void test() {
        while (isTest() && isEnabled()) {
        	//Mecanum drive using 2 joysticks
        	m_robotDrive.driveCartesian(m_rightJoystick.getX(), -m_rightJoystick.getY(), m_leftJoystick.getX(),0.0);
        	
        	//Slide rail controls
        	if(m_darioJoystick.getRawButton(3)) {
        		moveSlideUp(0.25);
        	}else if(m_darioJoystick.getRawButton(2)) {
        		moveSlideDown(0.25);
        	}else if(!m_darioJoystick.getRawButton(3) || !m_darioJoystick.getRawButton(2)) {
        		stopSlide(false);
        	}
        	
        	//Cube grabbing controls
        	if(m_darioJoystick.getTrigger()) {
        		grabCube(false);
        	}else if(m_darioJoystick.getRawButton(4)) {
        		releaseCube(false);
        	}else if(!m_darioJoystick.getTrigger() || !m_darioJoystick.getRawButton(4)) {
        		stopCube(false);
        	}
            Timer.delay(0.02);
        }
        
        
    }
    
    //Functions
    
    /**
     * Moves the robot forward a set distance in units.
     * 
     * @param distance Distance, in units.
     */
    public void moveForward(double distance) {
        if(Timer.getMatchTime() > 0.5){
            double initDistance = m_robotEncoder.getDistance();
    	    while(m_robotEncoder.getDistance() < initDistance + distance) {
    		m_robotDrive.driveCartesian(0.0, 0.25, 0.0, 0.0);
    		Timer.delay(0.02);
    	   }
    	   m_robotDrive.driveCartesian(0.0, -0.1, 0.0, 0.0);
    	   Timer.delay(0.02);
    	   m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
        }
    }
    
    /**
     * Moves the robot backward a set distance in units.
     * 
     * @param distance Distance, in units.
     */
    public void moveBackward(double distance) {
        if(Timer.getMatchTime() > 0.5){
            double initDistance = m_robotEncoder.getDistance();
    	    while(m_robotEncoder.getDistance() > initDistance - distance) {
      		    m_robotDrive.driveCartesian(0.0, -0.25, 0.0, 0.0);
    		  Timer.delay(0.02);
    	   }  
    	   m_robotDrive.driveCartesian(0.0, 0.1, 0.0, 0.0);
    	   Timer.delay(0.02);
    	   m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
        }
    }
    
    /**
     * Strafes the robot to the right a set distance.
     * @param distance Distance to strafe.
     */
    public void strafeRight(double distance) {//DOES NOT WORK
    	if(Timer.getMatchTime() > 0.5) {
    		m_robotEncoder.reset();
    		double initX = m_robotEncoder.getPositionX();
    		  while(m_robotEncoder.getPositionX() < initX + distance) {
        		    m_robotDrive.driveCartesian(0.25, 0.0, 0.0, 0.0);
        		    Timer.delay(0.02);
    		  }
    		  m_robotDrive.driveCartesian(-0.1, 0.0, 0.0, 0.0);
    		  Timer.delay(0.02);
    		  m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    	}  
    }
   
    /**
     * Strafes the robot to the left a set distance.
     * @param distance Distance to strafe.
     */
    public void strafeLeft(double distance) {//DOES NOT WORK
    	if(Timer.getMatchTime() > 0.5) {
    		m_robotEncoder.reset();
    		double initX = m_robotEncoder.getPositionX();
    		  while(m_robotEncoder.getPositionX() > initX - distance) {
        		    m_robotDrive.driveCartesian(-0.25, 0.0, 0.0, 0.0);
        		    Timer.delay(0.02);
    		  }
    		  m_robotDrive.driveCartesian(0.1, 0.0, 0.0, 0.0);
    		  Timer.delay(0.02);
    		  m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    	}  
    }  
    
    /**
     * Turns the robot to the right 90 degrees
     */
    public void turnRight() {
    	if(Timer.getMatchTime() > 0.5) {
    		double initBearing = onboardGyro.getAngle();
        	while(onboardGyro.getAngle() < initBearing + 90) {
        		System.out.println("Gyro: " + onboardGyro.getAngle());
        		m_robotDrive.driveCartesian(0.0, 0.0, 0.25, 0.0);
        		Timer.delay(0.02);
        	}
        	m_frontLeft.set(-0.1);
    		m_rearLeft.set(-0.1);
    		m_frontRight.set(0.1);
    		m_rearRight.set(0.1);
        	m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    	}
    }
    
    /**
     * Turns the robot to the left 90 degrees.
     */
    public void turnLeft() {
        if(Timer.getMatchTime() > 0.5){
            double initBearing = onboardGyro.getAngle();
    	while(onboardGyro.getAngle() > initBearing - 90) {
    		System.out.println("Gyro: " + onboardGyro.getAngle());
    		m_robotDrive.driveCartesian(0.0, 0.0, -0.25, 0.0);
    		Timer.delay(0.02);
    	}
    	m_frontLeft.set(0.1);
		m_rearLeft.set(0.1);
		m_frontRight.set(-0.1);
		m_rearRight.set(-0.1);
    	m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
        }
    }
    
    //Slide rail controls
    /**
     * Moves the slide up at a set speed.
     * @param speed Motor speed
     */
    public void moveSlideUp(double speed) {
 	   m_slideMotor1.set(speed);
 	   m_slideMotor2.set(-speed);
    }
    
    /**
     * Moves the slide down a set speed and time.
     * @param speed Motor speed
     * @param time Time to move slide
     */
    public void moveSlideUp(double speed, double time) {
    	if(Timer.getMatchTime() > 0.5) {
    		moveSlideUp(speed);
    		Timer.delay(time);
    		stopSlide(true);
    	}
    }
    
    /**
     * Moves the slide down at a set speed.
     * @param speed Motor speed
     */
    public void moveSlideDown(double speed) {
 	   m_slideMotor1.set(-speed);
 	   m_slideMotor2.set(speed);
    }
    
    /**
     * Moves the slide down at a set speed and time.
     * @param speed Motor speed
     * @param time Time to move slide
     */
    public void moveSlideDown(double speed, double time) {
    	if(Timer.getMatchTime() > 0.5) {
    		moveSlideDown(speed);
    		Timer.delay(time);
        	stopSlide(true);
    	}
    }
    
    /**
     * Stops the slide.
     */
    public void stopSlide(boolean autonomous) {
       if(autonomous) {
    	   if(Timer.getMatchTime() > 0.5) {
    		   m_slideMotor1.set(0.0);
    	 	   m_slideMotor2.set(0.0);
    	   }
       }else if(!autonomous) {
    	   m_slideMotor1.set(0.0);
     	   m_slideMotor2.set(0.0);
       }
    }
    
    //Pneumatic controls
    /**
     * Operates pneumatics in order to grab a cube.
     * @param autonomous whether being run in autonomous or teleop.
     */
    public void grabCube(boolean autonomous) {
        if(autonomous){
            if(Timer.getMatchTime() > 0.5){
                armSolenoid.set(DoubleSolenoid.Value.kForward);
            }else if(!autonomous){
            armSolenoid.set(DoubleSolenoid.Value.kForward);
            }
        }
    }
    
    /**
     * Operates pneumatics in order to release a cube.
     * @param autonomous whether being run in autonomous or teleop.
     */
    public void releaseCube(boolean autonomous) {
        if(autonomous){
            if(Timer.getMatchTime() > 0.5){
                armSolenoid.set(DoubleSolenoid.Value.kReverse);
            }
        }else if(!autonomous){
            armSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    	
    }
    
    /**
     * Stops arm solenoid operation.
     */
    public void stopCube(boolean autonomous){
    	if(autonomous) {
    		if(Timer.getMatchTime() > 0.5) {
    			armSolenoid.set(DoubleSolenoid.Value.kOff);
    		}
    	}else if(!autonomous) {
    		armSolenoid.set(DoubleSolenoid.Value.kOff);
    	}
    }
}