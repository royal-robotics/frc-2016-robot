package org.usfirst.frc.team2522.robot;

import java.io.IOException;
import java.io.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class OperatorController
{
	// Operator Stick Buttons
	public static final int FIELD_SHOT_SPEED_BUTTON = 1;
	public static final int INTAKE_BUTTON = 2;
	public static final int FIELD_SHOT_ANGLE_BUTTON = 3;
	public static final int ZAXIS_SHOT_SPEED_BUTTON = 4;
	public static final int PICKUP_BUTTON = 5;
	public static final int LIP_SHOT_SPEED_BUTTON = 6;
	public static final int SPITOUT_BUTTON = 7;
	public static final int SHOOT_BUTTON = 8;
	public static final int WALL_SHOT_ANGLE_BUTTON = 9;
	public static final int WALL_SHOT_SPEED_BUTTON = 10;
	public static final int CLIMBER_LOCK_BUTTON = 11;

	// Operator Stick POV Values
	public static final int INTAKE_OUT_POV = 0;
	public static final int INTAKE_IN_POV = 180;
	public static final int CLIMBER_LOCK_POV = 90;
	public static final int CLIMBER_ANGLE_POV = 270;

	// Operator Stick Joystick Axis
	public static final AxisType CLIMBER_AXIS = AxisType.kY;
	public static final AxisType ARM_AXIS = AxisType.kThrottle;

	// Driver Stick Buttons
	public static final int TOGGLE_SHIFTER_BUTTON = 1;
	public static final int TRACK_TARGET_BUTTON = 2;
	public static final int DRIVE_MODE_CHANGE_BUTTON = 3;	// left stick
	public static final int TURN_AROUND_BUTTON = 3;			// right stick
	public static final int DRIVE_STRAIGHT_BUTTON = 4;
	public static final int SHOW_TARGETS_BUTTON = 8;		
	public static final int SHOW_IMAGE_FILTER_BUTTON = 9;
	
	// Arm Angles used by various functions
	public static final double CLIMB_ANGLE = 68.0;
			
	public static final double PICKUP_ANGLE = -12.0;		// TODO: comp bot is -12, this is apparently to high for the practice bot
	public static final double PICKUP_RPMS = -2000.0;		// 
	
	public static final double SPITOUT_ANGLE = -10.0;		// degrees
	public static final double SPITOUT_RPMS = 3500.0;		// 
	
	public static final double WALL_SHOT_ANGLE = 104.0;		// degrees
	public static final double WALL_SHOT_RPMS = 2500.0;		// 2500 on comp bot
	
	public static final double LIP_SHOT_ANGLE = 116.0;
	public static final double LIP_SHOT_RPMS = 2400.0;		// ??? 
	
	public static final double FIELD_SHOT_ANGLE = 60.0;
	public static final double FIELD_SHOT_RPMS = 2500.0;	// Practice Bot Value
	
	// Toggle values used to control the various button states.
	//
	static boolean pickupButtonToggle = false;
	static boolean operateArmButtonToggle = false;
	static boolean operateShooterButtonToggle = false;
	static boolean climberLockButtonToggle = false;

	// Used to output PID data for Arm for calibration purposes.
	//
	static PrintStream fw = null;
	static Timer pidControlTimer = new Timer();
	
		
	/**
	 * Checks the robot l state and activates the pickup system accordingly.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operatePickup(Robot robot)
	{
		if (!robot.ballLoadedSwitch.get())
		{
			robot.ballLoaded = true;	
		}
		
		if (robot.operatorstick.getPOV(0) == INTAKE_OUT_POV){
			robot.intake.set(DoubleSolenoid.Value.kForward);
		}
		else if (robot.operatorstick.getPOV(0) == INTAKE_IN_POV)
		{
			robot.intake.set(DoubleSolenoid.Value.kReverse);
		}

		if (robot.operatorstick.getRawButton(LIP_SHOT_SPEED_BUTTON) || 
			robot.operatorstick.getRawButton(WALL_SHOT_SPEED_BUTTON) ||
			robot.operatorstick.getRawButton(FIELD_SHOT_SPEED_BUTTON))
		{
			if (!pickupButtonToggle)
			{
				robot.roller.set(0.0);
				robot.intake.set(DoubleSolenoid.Value.kReverse);
				pickupButtonToggle = true;
			}
		}
		else if(robot.operatorstick.getRawButton(PICKUP_BUTTON)) {
			if (!pickupButtonToggle)
			{
				robot.roller.set(-0.5);
				robot.intake.set(DoubleSolenoid.Value.kForward);
				pickupButtonToggle = true;
			}
		}
		else if(robot.operatorstick.getRawButton(SPITOUT_BUTTON))
		{
			if (!pickupButtonToggle)
			{
				robot.roller.set(0.5);
				robot.intake.set(DoubleSolenoid.Value.kReverse);
				pickupButtonToggle = true;
			}	
		}
		else if (pickupButtonToggle) {
			robot.roller.set(0);
			pickupButtonToggle = false;
		}
	}

	
	/**
	 * Operate the shooter arm.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operateArm(Robot robot)
	{
		double axisValue = robot.operatorstick.getAxis(ARM_AXIS); 
    	
		if (robot.operatorstick.getRawButton(WALL_SHOT_ANGLE_BUTTON) || 
			robot.operatorstick.getRawButton(PICKUP_BUTTON) || 
			robot.operatorstick.getRawButton(SPITOUT_BUTTON) ||
			robot.operatorstick.getRawButton(FIELD_SHOT_ANGLE_BUTTON)||
			robot.operatorstick.getPOV(0) == CLIMBER_ANGLE_POV			
			)
		{
			if(!operateArmButtonToggle)
			{
				try
				{
					fw = new PrintStream(new File("/home/lvuser/PIDValues.txt"));
					pidControlTimer.reset();
					pidControlTimer.start();
				}
				catch(IOException e)
				{
					fw = null;
					e.printStackTrace();
				}
				
				if (robot.operatorstick.getRawButton(WALL_SHOT_ANGLE_BUTTON)) {
					robot.armController.setTargetAngle(WALL_SHOT_ANGLE);
				}
				else if (robot.operatorstick.getRawButton(PICKUP_BUTTON)) {
					robot.armController.setTargetAngle(PICKUP_ANGLE);
				}
				else if (robot.operatorstick.getRawButton(SPITOUT_BUTTON)) {
					robot.armController.setTargetAngle(SPITOUT_ANGLE);
				}
				else if (robot.operatorstick.getRawButton(FIELD_SHOT_ANGLE_BUTTON)) {
					robot.armController.setTargetAngle(FIELD_SHOT_ANGLE);
				}
				else if (robot.operatorstick.getPOV(0) == CLIMBER_ANGLE_POV) {
					robot.armController.setTargetAngle(CLIMB_ANGLE);
				}
				
				operateArmButtonToggle = true;	
			}
			
			if (fw != null)
			{
				System.out.println(String.valueOf(pidControlTimer.get()) + "," + String.valueOf(robot.armAngle.pidGet()) + "," + String.valueOf(robot.armController.getSetpoint()));
				fw.println(String.valueOf(pidControlTimer.get()) + "," + String.valueOf(robot.armAngle.pidGet()) + "," + String.valueOf(robot.armController.getSetpoint()));
			}
		}
		else if (axisValue > 0.1 || axisValue < -0.1)
		{
			operateArmButtonToggle = true;
			if (axisValue > 0.1) {
				robot.armController.setSetpoint(robot.armAngle.pidGet() + (-50.0 * ArmController.VOLTS_PER_DEGREE * (axisValue * axisValue)));
			}
			else if (axisValue < -0.1) {
				robot.armController.setSetpoint(robot.armAngle.pidGet() + ( 50.0 * ArmController.VOLTS_PER_DEGREE * (axisValue * axisValue)));
			}
		}
		else if (operateArmButtonToggle)
		{
			robot.armController.setSetpoint(robot.armAngle.pidGet());
			operateArmButtonToggle = false;
			
			if (fw != null)
			{
				fw.close();
				fw = null;
			}
		}
	}
	
	/**
	 * Operate the shooter mechanism.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operateShooter(Robot robot)
	{
		if(robot.operatorstick.getRawButton(LIP_SHOT_SPEED_BUTTON) || 
		   robot.operatorstick.getRawButton(WALL_SHOT_SPEED_BUTTON) || 
		   robot.operatorstick.getRawButton(ZAXIS_SHOT_SPEED_BUTTON) ||
		   robot.operatorstick.getRawButton(FIELD_SHOT_SPEED_BUTTON) ||
		   robot.operatorstick.getRawButton(SPITOUT_BUTTON))
		{
			if (!operateShooterButtonToggle)
			{
				operateShooterButtonToggle = true;
			}

			// Only operate the kicker if one of the shooter speed buttons is set.
			//
			if (robot.operatorstick.getRawButton(SHOOT_BUTTON))
			{
				robot.kicker.set(DoubleSolenoid.Value.kForward);
				robot.ballLoaded = false;
			}
			else
			{
				robot.kicker.set(DoubleSolenoid.Value.kReverse);
			}

			// Set the shooter motor speed based on which button is selected.
			//
			if(robot.operatorstick.getRawButton(WALL_SHOT_SPEED_BUTTON)) {
				if (robot.getShooterTargetRPM() != WALL_SHOT_RPMS)
				{
					robot.setShooterTargetRPM(WALL_SHOT_RPMS);
				}
			}
			else if (robot.operatorstick.getRawButton(LIP_SHOT_SPEED_BUTTON)){
				if (robot.getShooterTargetRPM() != LIP_SHOT_RPMS)
				{
					robot.setShooterTargetRPM(LIP_SHOT_RPMS);
				}
			}
			else if (robot.operatorstick.getRawButton(FIELD_SHOT_SPEED_BUTTON)){
				if (robot.getShooterTargetRPM() != FIELD_SHOT_RPMS)
				{
					robot.setShooterTargetRPM(FIELD_SHOT_RPMS);
				}
			}
			else if (robot.operatorstick.getRawButton(SPITOUT_BUTTON)){
				if (robot.getShooterTargetRPM() != SPITOUT_RPMS)
				{
					robot.setShooterTargetRPM(SPITOUT_RPMS);
				}
			}
			else if (robot.operatorstick.getRawButton(ZAXIS_SHOT_SPEED_BUTTON))
			{
				if (robot.useRPM)
				{
					double speed = SmartDashboard.getNumber("Target RPM", 2500.0);
					robot.leftShooterWheel.set(speed);
					robot.rightShooterWheel.set(-speed);
				}
				else
				{
					double power = SmartDashboard.getNumber("Target PWR", 0.50);
					robot.leftShooterWheel.set(power);
					robot.rightShooterWheel.set(-power);
				}
			}
		}
		else if (robot.operatorstick.getRawButton(PICKUP_BUTTON) || robot.operatorstick.getRawButton(INTAKE_BUTTON)) {
			if (!operateShooterButtonToggle)
			{
				robot.kicker.set(DoubleSolenoid.Value.kReverse);
				
				if (robot.getShooterTargetRPM() != PICKUP_RPMS)
				{
					robot.setShooterTargetRPM(PICKUP_RPMS);
				}
				
				operateShooterButtonToggle = true;
			}
		}
		else if (operateShooterButtonToggle) {
			robot.setShooterTargetRPM(0.0);
			robot.kicker.set(DoubleSolenoid.Value.kReverse);
			operateShooterButtonToggle = false;
		}
	}

	/**
	 * Operate the climber mechanism.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operateClimber(Robot robot)
	{
		// TODO: the climber motor should be controlled by a PIDController set in distance mode
		//
		//if (robot.climbLock.get() != DoubleSolenoid.Value.kForward) {
			double axisValue = robot.operatorstick.getAxis(CLIMBER_AXIS); 
			if (axisValue > 0.05) {
				robot.climber.set(axisValue);
			}
			else if (axisValue < -.05) {
				robot.climber.set(axisValue);
			}
			else {
				robot.climber.set(0);
			}
		//}
		
		if (robot.operatorstick.getPOV(0) == CLIMBER_LOCK_POV) {
			if (!climberLockButtonToggle)
			{
				climberLockButtonToggle = true;
				if (robot.climbLock.get() == DoubleSolenoid.Value.kReverse) {
					robot.climbLock.set(DoubleSolenoid.Value.kForward);
					robot.climber.set(0);
				}
				else {
					robot.climbLock.set(DoubleSolenoid.Value.kReverse);
				}
			}
		}
		else if (robot.operatorstick.getPOV(0) != CLIMBER_LOCK_POV) {
			climberLockButtonToggle = false;
		}
		// TODO: the climber lock needs to be tied to the dashboard timer also, so that it auto locks just before the match ends.
	}
}
