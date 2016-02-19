package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;


public class ArmController extends PIDController
{
	public static final double voltsPerdegree = 0.0034180790;
	public static final double homeAngle = 139.0;	// degrees
	public static final double floorAngle = -38.0;	// degrees
	public static final double radiansPerDegree = 3.1415 / 180.0;
	
	public static final double minPower = 0.10;	// This is the minimum motor power level needed to hold the arm at straight out at 0 degrees.
	
	private ArmMotor m_ArmMotor;
	
	public ArmController(ArmMotor armMotor, PIDSource armIndicator)
	{
		super(3.0, 0.05, 0.0, armIndicator, armMotor);
		m_ArmMotor = armMotor;
		this.setInputRange(armMotor.m_HomeValue - ((homeAngle - floorAngle) * voltsPerdegree), armMotor.m_HomeValue);
		this.setOutputRange(-1.0,  1.0);
	}
	
	/**
	 * Set the setpoint for the PIDController
	 * Clears the queue for GetAvgError().
	 *$
	 * @param degrees the desired setpoint in degrees
	 */
	public synchronized void setSetpoint(double degrees) {
		super.setSetpoint(m_ArmMotor.m_FullyExtededVoltage + (degrees * voltsPerdegree));
	}		  
		  
}
