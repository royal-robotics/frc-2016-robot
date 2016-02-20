package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmController extends PIDController
{
	public static final double homeAngle = 128.6;	// degrees
	public static final double floorAngle = -46.0;	// degrees
	public static final double pickupAngle = -6.0;	// degrees
	public static final double voltsPerDegree = 0.0049227246;
	public static final double radiansPerDegree = 3.1415 / 180.0;
	
	public static final double minPower = 0.35;	// This is the approximate minimum motor power level needed to hold the arm at straight out at 0 degrees.
	public static final double defaultHomeVoltage = 2.950;
	
	double homeVoltage = defaultHomeVoltage;
	double floorVoltage = defaultHomeVoltage - ((homeAngle - floorAngle) * voltsPerDegree);
	double fullyExtendedVoltage = defaultHomeVoltage - (homeAngle * voltsPerDegree);
	
	PIDOutput armMotor;
	PIDSource armSensor;
	
	public ArmController(PIDSource armSensor, PIDOutput armMotor)
	{
	//	super(2.0, 0.05, 0.0, armSensor, armMotor);
		super(1.7, 0.005, 0.02, armSensor, armMotor);
		this.armMotor = armMotor;
		this.armSensor = armSensor;
		this.setOutputRange(-.65,  0.65);
		this.setInputRange(this.floorVoltage, this.homeVoltage);
		this.setPercentTolerance(0.5);
	}
	
	/**
	 * Call this function when home limit switch is hit.
	 * 
	 * @param voltage the voltage to use for the fully up / home position.
	 */
	public void setHomeVoltage(double voltage)
	{
		this.homeVoltage = voltage;
		this.floorVoltage = this.homeVoltage - ((homeAngle - floorAngle) * voltsPerDegree);
		this.fullyExtendedVoltage = this.homeVoltage - (homeAngle * voltsPerDegree);
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
		this.setSetpoint(this.fullyExtendedVoltage + (degrees * voltsPerDegree));
	}
	
	/**
	 * Get the current setpoint for the ArmController in terms of degrees.
	 * 
	 * @return the target setpoint value in degrees.
	 */
	public  double getTargetAngle()
	{
		return (this.getSetpoint() - this.fullyExtendedVoltage) / voltsPerDegree;
	}

	/**
	 * Get the calculated current angle of the shooter arm in degrees.
	 * Zero should be pointing straight out.
	 * 
	 * @return The current calculated angle of the shooter in degrees.
	 */
	public double getAngle()
	{
		return (this.armSensor.pidGet() - this.fullyExtendedVoltage) / voltsPerDegree;
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
		
		if (this.armSensor == null || this.armMotor == null) {
			return;
		}
		
		double offset = 0.0;

		offset = minPower * Math.cos(this.getAngle() * radiansPerDegree);
			
		this.armMotor.pidWrite(this.get() + offset);
	}

}
