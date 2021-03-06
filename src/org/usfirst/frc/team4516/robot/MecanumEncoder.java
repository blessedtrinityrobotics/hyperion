/*
 * Class used to enhance encoder methods and functions in a Mecanum-style drive train.
 * @Authors: Carlos Saucedo and Juan Carlos Santamaria
 */

package org.usfirst.frc.team4516.robot;
import edu.wpi.first.wpilibj.Encoder;
public class MecanumEncoder{
	//Encoder variables
	private Encoder m_frontRight;
	private Encoder m_rearRight;
	private Encoder m_frontLeft;
	private Encoder m_rearLeft;
	private double m_positionX;
	private double m_positionY;
	
	public MecanumEncoder(Encoder frontRight, Encoder rearRight, Encoder frontLeft, Encoder rearLeft) {
		m_frontRight = frontRight;
		m_rearRight = rearRight;
		m_frontLeft = frontLeft;
		m_rearLeft = rearLeft;
		m_positionX = 0;
		m_positionY = 0;
	}
	
	
	/**
	 * Resets encoders and positionX and positionY values.
	 */
	public void reset() {
		m_positionX = 0;
		m_positionY = 0;
		m_frontRight.reset();
		m_rearRight.reset();
		m_frontLeft.reset();
		m_rearLeft.reset();
		
	}
	
	public double getVelocityX() {
		double velocity = ((m_frontRight.getRate() + m_rearRight.getRate() + m_frontLeft.getRate() + m_rearLeft.getRate())/4.0);
		return(velocity);
	}
	
	public double getDistanceY() {
		double dY = ((m_frontRight.getDistance() + m_rearRight.getDistance() + m_frontLeft.getDistance() + m_rearLeft.getDistance())/4.0);
		return(dY);
	}
	
	public double getVelocityY() {
		double velocity = ((m_frontLeft.getRate() - m_frontRight.getRate() - m_rearLeft.getRate() + m_rearRight.getRate())/4.0);
		return(velocity);
	}
	
	public double getDistanceX() {
		double dX = ((m_frontLeft.getDistance() - m_frontRight.getDistance() - m_rearLeft.getDistance() + m_rearRight.getDistance())/4.0);
		return(dX);
	}
	
	/**
	 * Updates the current position of the robot given the current heading.
	 * @param robotHeading Robot heading in degrees.
	 */
	
	public void updatePosition(double robotHeading) {
		//Compute change in Distance with the robot as FoR
		double robotDeltaX = (getDistanceX());
		double robotDeltaY = (getDistanceY());
		m_positionX += robotDeltaX * Math.sin(robotHeading / 180.0 * Math.PI);
		m_positionY += robotDeltaX * Math.cos(robotHeading / 180.0 * Math.PI);
		m_positionX += robotDeltaY * Math.cos(robotHeading / 180.0 * Math.PI);
		m_positionY -= robotDeltaY * Math.sin(robotHeading / 180.0 * Math.PI);
	}
	
	/**
	 * 
	 * @return X coordinate of robot in world Cartesian.
	 */
	
	public double getPositionX() {
		return m_positionX;
	}	
	
	/**
	 * 
	 * @return Y coordinate of robot in world Cartesian.
	 */
	
	public double getPositionY() {
		return m_positionY;
	}
	
	public double getDistance() {
		double distance = ((m_frontRight.getDistance() + m_rearRight.getDistance() + m_frontLeft.getDistance() + m_rearLeft.getDistance()) / 4 );
		return distance;
	}
	
}
