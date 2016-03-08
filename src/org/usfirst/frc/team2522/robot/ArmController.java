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
	public static final double FLOOR_ANGLE = -41.0;			  
	public static final double FLOOR_DEFAULT_VOLTAGE = 2.668;

	// Competition Robot Values
	//
	//public static final double HOME_ANGLE = 118.9;
	//public static final double HOME_DEFAULT_VOLTAGE = 3.190;
	//public static final double FLOOR_ANGLE = -41.0;			  
	//public static final double FLOOR_DEFAULT_VOLTAGE = 2.319;
	
	public static final double VOLTS_PER_DEGREE = (HOME_DEFAULT_VOLTAGE - FLOOR_DEFAULT_VOLTAGE) / (HOME_ANGLE - FLOOR_ANGLE);
	public static final double RAIDIANS_PER_DEGREE = 3.1415 / 180.0;
	public static final double MIN_POWER = 0.35;	// This is the approximate minimum motor power level needed to hold the arm straight out at 0 degrees.

	
	double homeVoltage = HOME_DEFAULT_VOLTAGE;
	double floorVoltage = FLOOR_DEFAULT_VOLTAGE;
	double fullyExtendedVoltage = HOME_DEFAULT_VOLTAGE - (HOME_ANGLE * VOLTS_PER_DEGREE);
	
	PIDOutput armMotor;
	PIDSource armSensor;
	
	public ArmController(PIDSource armSensor, PIDOutput armMotor)
	{
		super(1.0, 0.04, 0.2, armSensor, armMotor);
//		super(0.8, 0.04, 0.35, armSensor, armMotor);
//		super(1.00, 0.1, 0.05, armSensor, armMotor);
		this.armMotor = armMotor;
		this.armSensor = armSensor;
		this.setOutputRange(-1 + MIN_POWER,  1.0 - MIN_POWER);
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
