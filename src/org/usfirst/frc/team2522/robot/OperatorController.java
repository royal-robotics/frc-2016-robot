package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class OperatorController
{
//	public static final int CLIMBER_RETRACT_BUTTON = 1;
//	public static final int ARM_DOWN_BUTTON = 2;
//	public static final int CLIMBER_DEPLOY_BUTTON = 3;
	public static final int ZAXIS_SHOT_SPEED_BUTTON = 4;
	public static final int PICKUP_BUTTON = 5;
	public static final int LIP_SHOT_SPEED_BUTTON = 6;
	public static final int SPITOUT_BUTTON = 7;
	public static final int SHOOT_BUTTON = 8;
	public static final int SHOOTER_POS_BUTTON = 9;
	//public static final int CALIBRATE_MOTOR_BUTTON = 10;
	public static final int WALL_SHOT_SPEED_BUTTON = 10;
	public static final int CLIMBER_LOCK_BUTTON = 11;
	
	public static final int INTAKE_OUT_POV = 0;
	public static final int INTAKE_IN_POV = 180;
	public static final int CLIMBER_LOCK_POV = 90;

	public static final AxisType CLIMBER_AXIS = AxisType.kY;
	public static final AxisType ARM_AXIS = AxisType.kThrottle;
	
	// Arm Angles used by various functions
	public static final double CLOSE_SHOT_ANGLE = 109.0;		// degrees
	public static final double OUTERWORKS_SHOT_ANGLE = 58.0;	// degrees
	public static final double PICKUP_ANGLE = -15.0;			// degrees
	public static final double SPITOUT_ANGLE = -15.0;			// degrees
	public static final double LOW_GOAL_TRAVERSE_ANGLE = -15.0;	// degrees
	
	public static final double PICKUP_SPEED = 0.45;
	public static final double SPITOUT_SPEED = 0.45;
	public static final double WALL_SHOT_SPEED = 0.46;
	public static final double LIP_SHOT_SPEED = 0.5;
	
	// Used to stop the arm after arm move buttons are no longer being pressed.
	//
	static boolean pickupButtonToggle = false;
	static boolean operateArmButtonToggle = false;
	static boolean operateShooterButtonToggle = false;
	static boolean climberLockButtonToggle = false;
		
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

		if (robot.operatorstick.getRawButton(LIP_SHOT_SPEED_BUTTON) || robot.operatorstick.getRawButton(WALL_SHOT_SPEED_BUTTON))
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
    	
		if (robot.operatorstick.getRawButton(SHOOTER_POS_BUTTON) || robot.operatorstick.getRawButton(PICKUP_BUTTON) || robot.operatorstick.getRawButton(SPITOUT_BUTTON))
		{
			if(!operateArmButtonToggle)
			{
				if (robot.operatorstick.getRawButton(SHOOTER_POS_BUTTON)) {
					robot.armController.setTargetAngle(CLOSE_SHOT_ANGLE);
				}
				else if (robot.operatorstick.getRawButton(PICKUP_BUTTON)) {
					robot.armController.setTargetAngle(PICKUP_ANGLE);
				}
				else if (robot.operatorstick.getRawButton(SPITOUT_BUTTON)) {
					robot.armController.setTargetAngle(SPITOUT_ANGLE);
				}
				operateArmButtonToggle = true;	
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
		}
	}
	
	/**
	 * Operate the shooter mechanism.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operateShooter(Robot robot)
	{
		if(robot.operatorstick.getRawButton(LIP_SHOT_SPEED_BUTTON) || robot.operatorstick.getRawButton(WALL_SHOT_SPEED_BUTTON) || robot.operatorstick.getRawButton(ZAXIS_SHOT_SPEED_BUTTON))
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

			if(robot.operatorstick.getRawButton(WALL_SHOT_SPEED_BUTTON)) {
				robot.leftShooterWheel.set(WALL_SHOT_SPEED);
				robot.rightShooterWheel.set(-WALL_SHOT_SPEED);
			}
			else if (robot.operatorstick.getRawButton(LIP_SHOT_SPEED_BUTTON)){
				robot.leftShooterWheel.set(LIP_SHOT_SPEED);
				robot.rightShooterWheel.set(-LIP_SHOT_SPEED);
			}
			else if (robot.operatorstick.getRawButton(ZAXIS_SHOT_SPEED_BUTTON))
			{
				robot.leftShooterWheel.set(robot.rightstick.getZ());
				robot.rightShooterWheel.set(-robot.rightstick.getZ());
			}
			if (robot.operatorstick.getRawButton(ZAXIS_SHOT_SPEED_BUTTON))
			{
				double speed = (robot.rightstick.getZ() + 1.0) / 2.0;
				robot.leftShooterWheel.set(speed);
				robot.rightShooterWheel.set(-speed);
			}
		}
		else if (robot.operatorstick.getRawButton(PICKUP_BUTTON)) {
			if (!operateShooterButtonToggle)
			{
				robot.kicker.set(DoubleSolenoid.Value.kReverse);
				robot.leftShooterWheel.set(-PICKUP_SPEED);
				robot.rightShooterWheel.set(PICKUP_SPEED);
				operateShooterButtonToggle = true;
			}
		}
		else if (robot.operatorstick.getRawButton(SPITOUT_BUTTON)) {
			if (!operateShooterButtonToggle)
			{
				robot.kicker.set(DoubleSolenoid.Value.kReverse);
				robot.leftShooterWheel.set(SPITOUT_SPEED);
				robot.rightShooterWheel.set(-SPITOUT_SPEED);
				operateShooterButtonToggle = true;
			}
		}
		else if (operateShooterButtonToggle) {
			robot.kicker.set(DoubleSolenoid.Value.kReverse);
			robot.leftShooterWheel.set(0);
			robot.rightShooterWheel.set(0);
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
		if (robot.climbLock.get() != DoubleSolenoid.Value.kForward) {
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
		}
		
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
	
	public static void calibrateMotor(Robot robot)
	{
		/*
    	if (robot.operatorstick.getRawButton(CALIBRATE_MOTOR_BUTTON))
    	{
			double motorPort = SmartDashboard.getNumber("Calibrate Motor");
			VictorSP motor = null;
			boolean valid = true;
			
			switch ((int)motorPort)
			{
				case 1:
					motor = robot.leftDrive;
					break;
				case 3:
					motor = robot.armMotor;
					break;
				case 5:
					motor = robot.rightDrive;
					break;
				case 6:
					motor = robot.climber;
					break;
				case 7:
					motor = robot.roller;
					break;
				default:
					valid = false;
					break;
			}
			
			if (valid) {
				motor.set(robot.operatorstick.getRawAxis(3));
			}
    	}
    	*/
	}
}
