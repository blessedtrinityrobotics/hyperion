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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;//Gyroscope
import edu.wpi.first.wpilibj.DoubleSolenoid;


@SuppressWarnings("deprecation")//Yeah I don't give a damn but I need to have this code done
public class Robot extends SampleRobot{
	public static final int solenoidChan1 = 0;
	public static final int solenoidChan2 = 1;
	
	public void robotInit() {
		
	}
	
	/**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() { 
    	
    }

    /**
     * This function is called once each time the robot enters teleop mode.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	
        }
    }
        	
        	

}