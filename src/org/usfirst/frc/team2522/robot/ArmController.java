package org.usfirst.frc.team2522.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;


public class ArmController extends PIDController
{

	// Practice Robot Values
	//
	//public static final double HOME_DEFAULT_VOLTAGE = 2.565; //Practice value
	//public static final double FLOOR_DEFAULT_VOLTAGE = 1.885;
	//public static final double HOME_ANGLE = 116.2;	// degrees
	//public static final double FLOOR_ANGLE = -49.5;			  

	// Competition Robot Values
	//
	public static final double HOME_DEFAULT_VOLTAGE = 3.010;
	public static final double FLOOR_DEFAULT_VOLTAGE = 2.363;
	public static final double HOME_ANGLE = 111.9;	// degrees
	public static final double FLOOR_ANGLE = -51.1;			  

	
	public static double VOLTS_PER_DEGREE = (HOME_DEFAULT_VOLTAGE - FLOOR_DEFAULT_VOLTAGE) / (HOME_ANGLE - FLOOR_ANGLE);
	public static final double RAIDIANS_PER_DEGREE = 3.1415 / 180.0;
	public static final double MIN_POWER = 0.00;	// This is the approximate minimum motor power level needed to hold the arm straight out at 0 degrees.

	
	double homeVoltage = HOME_DEFAULT_VOLTAGE;
	double floorVoltage = FLOOR_DEFAULT_VOLTAGE;
	double fullyExtendedVoltage = HOME_DEFAULT_VOLTAGE - (HOME_ANGLE * VOLTS_PER_DEGREE);
	
	// Used to output PID data for Arm for calibration purposes.
	//
	PrintStream fw = null;
	Timer pidControlTimer = new Timer();
	
	PIDOutput armMotor;
	PIDSource armSensor;
	
	public ArmController(PIDSource armSensor, PIDOutput armMotor)
	{

		super(15.0, 0.0, 6.0, armSensor, armMotor);
		//super(20.0, 0.0, 8.0, armSensor, armMotor);// work in progress
//		super(5.0, 0.0, 0.75, armSensor, armMotor);	// work in progress
//		super(6.0, 0.5, 2.5, armSensor, armMotor);
		this.armMotor = armMotor;
		this.armSensor = armSensor;
		this.setOutputRange(-1.0 + MIN_POWER,  1.0 - MIN_POWER);
		this.setInputRange(this.floorVoltage, this.homeVoltage);
		this.setPercentTolerance(0.5);
	}
	
	/***
	 * 
	 */
	public void startTrace()
	{
		try
		{
			fw = new PrintStream(new File("/home/lvuser/ArmPIDValues.txt"));
			pidControlTimer.reset();
			pidControlTimer.start();
		}
		catch(IOException e)
		{
			fw = null;
			e.printStackTrace();
		}
	}
	
	/***
	 * 
	 */
	public void stopTrace()
	{
		if (fw != null)
		{
			pidControlTimer.stop();
			pidControlTimer.reset();
			fw.close();
			fw = null;
		}
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
			
			// If this is not null then tracing is enabled and we want to write values out to a file.
			if (fw != null)
			{
				fw.println(String.valueOf(pidControlTimer.get()) + "," + String.valueOf(this.armSensor.pidGet()) + "," + String.valueOf(this.getSetpoint()));
			}
		}
	}

}
