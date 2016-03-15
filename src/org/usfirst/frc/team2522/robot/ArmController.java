package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;


public class ArmController extends PIDController
{

	// Practice Robot Values
	//
	public static final double HOME_DEFAULT_VOLTAGE = 3.615; //Practice value
	public static final double FLOOR_DEFAULT_VOLTAGE = 2.753;

	// Competition Robot Values
	//
	//public static final double HOME_DEFAULT_VOLTAGE = 2.671;
	//public static final double FLOOR_DEFAULT_VOLTAGE = 1.841;

	public static final double HOME_ANGLE = 116;	// degrees
	public static final double FLOOR_ANGLE = -38;			  
	
	public static double VOLTS_PER_DEGREE = (HOME_DEFAULT_VOLTAGE - FLOOR_DEFAULT_VOLTAGE) / (HOME_ANGLE - FLOOR_ANGLE);
	public static final double RAIDIANS_PER_DEGREE = 3.1415 / 180.0;
	public static final double MIN_POWER = 0.00;	// This is the approximate minimum motor power level needed to hold the arm straight out at 0 degrees.

	
	double homeVoltage = HOME_DEFAULT_VOLTAGE;
	double floorVoltage = FLOOR_DEFAULT_VOLTAGE;
	double fullyExtendedVoltage = HOME_DEFAULT_VOLTAGE - (HOME_ANGLE * VOLTS_PER_DEGREE);
	
	PIDOutput armMotor;
	PIDSource armSensor;
	
	public ArmController(PIDSource armSensor, PIDOutput armMotor)
	{
		super(12.0, 0.5, 0.5, armSensor, armMotor);
		this.armMotor = armMotor;
		this.armSensor = armSensor;
		this.setOutputRange(-1 + MIN_POWER,  1.0 - MIN_POWER);
		this.setInputRange(this.floorVoltage, this.homeVoltage);
		this.setPercentTolerance(0.5);
	}
	
	/**
	 * Set the setpoint for the PIDController to the specified angle in degrees
	 * Clears the queue for GetAvgError().
	 *$
	 * @param degrees the desired setpoint in degrees
	 */
	public  void setTargetAngle(double degrees)
	{
		this.setSetpoint(this.fullyExtendedVoltage + (degrees * VOLTS_PER_DEGREE));
	}
	
	/**
	 * Get the current setpoint for the ArmController in terms of degrees.
	 * 
	 * @return the target setpoint value in degrees.
	 */
	public  double getTargetAngle()
	{
		return (this.getSetpoint() - this.fullyExtendedVoltage) / VOLTS_PER_DEGREE;
	}

	/**
	 * Get the calculated current angle of the shooter arm in degrees.
	 * Zero should be pointing straight out.
	 * 
	 * @return The current calculated angle of the shooter in degrees.
	 */
	public double getAngle()
	{
		return (this.armSensor.pidGet() - this.fullyExtendedVoltage) / VOLTS_PER_DEGREE;
	}
	
	/**
	 * 
	 */
	@Override
	public synchronized void setSetpoint(double setpoint) {
		if (setpoint > this.homeVoltage) {
			setpoint = this.homeVoltage;
		}
		
		if (setpoint < this.floorVoltage) {
			setpoint = this.floorVoltage;
		}
		
		super.setSetpoint(setpoint);
	}
	
	/**
	 * 
	 */
	@Override
	protected void calculate() {
		super.calculate();
		
		if (this.isEnabled())
		{
			if (this.armSensor == null || this.armMotor == null) {
				return;
			}
			
			double offset = 0.0;
	
			offset = MIN_POWER * Math.cos(this.getAngle() * RAIDIANS_PER_DEGREE);
							
			this.armMotor.pidWrite(-1.0 * (this.get() + offset));
		}
	}

}
