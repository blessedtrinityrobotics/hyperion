package org.usfirst.frc.team4516.robot;
import edu.wpi.first.wpilibj.Encoder;
import java.math.*;
public class MecanumEncoder{
	//Encoder vars
	private Encoder frontRight;
	private Encoder rearRight;
	private Encoder frontLeft;
	private Encoder rearLeft;
	private double m_positionX;
	private double m_positionY;
	
	public MecanumEncoder(Encoder frontRight, Encoder rearRight, Encoder frontLeft, Encoder rearLeft) {
		frontRight = this.frontRight;
		rearRight = this.rearRight;
		frontLeft = this.frontLeft;
		rearLeft = this.rearLeft;
		m_positionX = 0;
		m_positionY = 0;
	}
	
	public void reset() {//Resets the positionX and positionY values.
		m_positionX = 0;
		m_positionY = 0;
	}
	
	public double getVelocityX() {
		double velocity = ((frontRight.getRate() + rearRight.getRate() + frontLeft.getRate() + rearLeft.getRate())/4.0);
		return(velocity);
	}
	
	public double getVelocityY() {
		double velocity = ((frontLeft.getRate() - frontRight.getRate() - rearLeft.getRate() + rearRight.getRate())/4.0);
		return(velocity);
	}
	
	/**
	 * Updates the current position of the robot given the current heading and change in time.
	 * 
	 * @param changeInTime Elapsed time in seconds.
	 * @param robotHeading Robot heading in degrees.
	 */
	public void updatePosition(double changeInTime, double robotHeading) {
		//Compute change in Distance with the robot as FoR
		double robotDeltaX = (getVelocityX() * changeInTime);
		double robotDeltaY = (getVelocityY() * changeInTime);
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
	
	
	
}
