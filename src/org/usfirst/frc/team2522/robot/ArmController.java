package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;


public class ArmController extends PIDController
{
	// Practice Robot Values
	//
	public static final double HOME_ANGLE = 118.4;	// degrees
	public static final double HOME_DEFAULT_VOLTAGE = 3.557; //Practice value
	public static final double VOLTS_PER_DEGREE = 0.005575495; //Practice value

	// Competition Robot Values
	//
	//public static final double HOME_ANGLE = 118.9;	// degrees
	//public static final double HOME_DEFAULT_VOLTAGE = 3.101; //Competition value
	//public static final double VOLTS_PER_DEGREE = 0.0054478527; //Competition value

	
	public static final double FLOOR_ANGLE = -41.0;
	public static final double RAIDIANS_PER_DEGREE = 3.1415 / 180.0;
	public static final double MIN_POWER = 0.35;	// This is the approximate minimum motor power level needed to hold the arm at straight out at 0 degrees.

	
	double homeVoltage = HOME_DEFAULT_VOLTAGE;
	double floorVoltage = HOME_DEFAULT_VOLTAGE - ((HOME_ANGLE - FLOOR_ANGLE) * VOLTS_PER_DEGREE);
	double fullyExtendedVoltage = HOME_DEFAULT_VOLTAGE - (HOME_ANGLE * VOLTS_PER_DEGREE);
	
	PIDOutput armMotor;
	PIDSource armSensor;
	
	public ArmController(PIDSource armSensor, PIDOutput armMotor)
	{
//		super(2.0, 0.005, 0.02, armSensor, armMotor);
		super(2.0, 0.005, 0.02, armSensor, armMotor);
		this.armMotor = armMotor;
		this.armSensor = armSensor;
		this.setOutputRange(-1 + MIN_POWER,  1.0 - MIN_POWER);
		this.setInputRange(this.floorVoltage, this.homeVoltage);
		this.setPercentTolerance(.5);
	}
	
	/**
	 * Call this function when home limit switch is hit.
	 * 
	 * @param voltage the voltage to use for the fully up / home position.
	 */
	public void setHomeVoltage(double voltage)
	{
		this.homeVoltage = voltage;
		this.floorVoltage = this.homeVoltage - ((HOME_ANGLE - FLOOR_ANGLE) * VOLTS_PER_DEGREE);
		this.fullyExtendedVoltage = this.homeVoltage - (HOME_ANGLE * VOLTS_PER_DEGREE);
		this.setInputRange(this.floorVoltage, this.homeVoltage);
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
							
			this.armMotor.pidWrite(this.get() + offset);
		}
	}

}
