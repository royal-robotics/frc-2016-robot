package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class ArmController extends PIDController implements PIDOutput, PIDSource {

	public static final double voltsIn90Degrees = 0.2;
	public static final double degreesPerVolt = 90.0 / voltsIn90Degrees;
	public static final double homeAngle = 135.0;	// degrees
	public static final double floorAngle = -30.0;	// degrees
	public static final double radiansPerDegree = 3.1415 / 180.0;
	
	public static final double minPower = 0.35;	// This is the minimum motor power level needed to hold the arm at straight out at 0 degrees.
	
	private PIDSource  m_PIDSource;
	private PIDOutput m_PIDOutput;
	
	private double m_FullyExtededVoltage;
	
	public ArmController(PIDOutput armMotor, PIDSource armIndicator, double homeVoltage)
	{
		super(1.0, 0.01, 0.0, armIndicator, armMotor);

		m_PIDOutput = armMotor;
		m_PIDSource = armIndicator;

		m_FullyExtededVoltage = homeVoltage - (homeAngle / degreesPerVolt);
		this.setInputRange(homeVoltage - ((homeAngle - floorAngle) / degreesPerVolt), homeVoltage);
		this.setOutputRange(-1.0,  1.0);
	}
	
	/**
	 * Set the setpoint for the PIDController
	 * Clears the queue for GetAvgError().
	 *$
	 * @param degrees the desired setpoint in degrees
	 */
	public synchronized void setSetpoint(double degrees) {
		super.setSetpoint(m_FullyExtededVoltage + (degrees * degreesPerVolt));
	}		  
		  
	public void pidWrite(double output)
	{
		output += Math.cos((m_pidInput.pidGet() - m_FullyExtededVoltage) * scale);
		
		m_PIDOutput.pidWrite(output);
	}
	
	/**
	 * Set which parameter of the device you are using as a process control
	 * variable.
	 *
	 * @param pidSource
	 *            An enum to select the parameter.
	 */
	public void setPIDSourceType(PIDSourceType pidSource)
	{
		m_PIDSource.setPIDSourceType(pidSource);
	}

	/**
	 * Get which parameter of the device you are using as a process control
	 * variable.
	 *
	 * @return the currently selected PID source parameter
	 */
	public PIDSourceType getPIDSourceType()
	{
		return m_PIDSource.getPIDSourceType();
	}

	/**
	 * Get the result to use in PIDController
	 *$
	 * @return the result to use in PIDController
	 */
	public double pidGet()
	{
		return m_PIDSource.pidGet();
	}
}
