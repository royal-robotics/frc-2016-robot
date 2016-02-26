package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class OperatorController
{
	public static final int CLIMBER_DEPLOY_BUTTON = 1;
	public static final int ARM_DOWN_BUTTON = 2;
	public static final int CLIMBER_RETRACT_BUTTON = 3;
	public static final int ARM_UP_BUTTON = 4;
	public static final int PICKUP_BUTTON = 5;
	public static final int SPITOUT_BUTTON = 7;
	public static final int SHOOTER_POS_BUTTON = 9;
	public static final int SHOOTER_READY_BUTTON = 6;
	public static final int SHOOT_BUTTON = 8;
	public static final int CALIBRATE_MOTOR_BUTTON = 10;
	public static final int CLIMBER_LOCK_BUTTON = 11;

	// Arm Angles used by various functions
	public static final double PICKUP_ANGLE = -15.0;		// degrees
	public static final double OUTERWORKS_SHOT_ANGLE = 62.0;
	public static final double LOW_GOAL_TRAVERSE_ANGLE = -15.0;	// degrees
	
	// Used to stop the arm after arm move buttons are no longer being pressed.
	//
	static boolean armMoveButtonToggle = false;
	static boolean pickupButtonToggle = false;
	static boolean spitoutButtonToggle = false;
	static boolean shooterPosButtonToggle = false;
	static boolean shooterReadyButtonToggle = false;
	static boolean climberLockButtonToggle = false;
	static boolean rollerButtonToggle = false;
		
	/**
	 * Checks the robot control state and activates the pickup system accordingly.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operatePickup(Robot robot)
	{
		if ((robot.operatorstick.getPOV(0) == 0) || (robot.leftstick.getRawButton(1))){
			robot.intake.set(DoubleSolenoid.Value.kForward);
		}
		else if (robot.operatorstick.getPOV(0) == 180)
		{
			robot.intake.set(DoubleSolenoid.Value.kReverse);
		}
		
		if (robot.leftstick.getRawButton(1)) {
			if (!rollerButtonToggle) {
				if (robot.intake.get() == DoubleSolenoid.Value.kReverse) {
					robot.intake.set(DoubleSolenoid.Value.kForward);
				}
				else {
					robot.intake.set(DoubleSolenoid.Value.kReverse);
				}
				rollerButtonToggle = true;
			}
		}
		else {
			rollerButtonToggle = false;
		}
		
		if(robot.operatorstick.getRawButton(PICKUP_BUTTON) && ! robot.operatorstick.getRawButton(SHOOTER_READY_BUTTON)) {
			if (!pickupButtonToggle)
			{
				robot.roller.set(-0.5);
				robot.intake.set(DoubleSolenoid.Value.kForward);
				robot.leftShooterWheel.set(-0.4);
				robot.rightShooterWheel.set(0.4);
				robot.armController.setTargetAngle(PICKUP_ANGLE);
				pickupButtonToggle = true;
			}
		} else if(robot.operatorstick.getRawButton(SPITOUT_BUTTON) && ! robot.operatorstick.getRawButton(SHOOTER_READY_BUTTON)) {
			if (!spitoutButtonToggle) {
				robot.roller.set(0.5);
				robot.intake.set(DoubleSolenoid.Value.kReverse);
				robot.leftShooterWheel.set(0.4);
				robot.rightShooterWheel.set(-0.4);
				robot.armController.setTargetAngle(PICKUP_ANGLE);
				spitoutButtonToggle = true;
			}	
		}
		else if (pickupButtonToggle || spitoutButtonToggle) {
			robot.roller.set(0);
			robot.leftShooterWheel.set(0);
			robot.rightShooterWheel.set(0);
			
			pickupButtonToggle = false;
			spitoutButtonToggle = false;
		}
	}
	
	/**
	 * Operate the shooter arm.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operateArm(Robot robot) {
		
    	if (robot.shooterHome.get()) {
    		robot.armController.setHomeVoltage(robot.armAngle.pidGet());
    	}
    	
		if (robot.operatorstick.getRawButton(SHOOTER_POS_BUTTON)) {
			if(!shooterPosButtonToggle) {
				robot.armController.setTargetAngle(62.0);
				shooterPosButtonToggle = true;	
			}
		}
		else if (shooterPosButtonToggle) {
			robot.armController.setSetpoint(robot.armAngle.pidGet());
			shooterPosButtonToggle = false;
		}
		
		if(robot.operatorstick.getRawButton(ARM_UP_BUTTON)){
			robot.armController.setSetpoint(robot.armAngle.pidGet() + ArmController.VOLTS_PER_DEGREE * 35.0);
			armMoveButtonToggle = true;
		} else if(robot.operatorstick.getRawButton(ARM_DOWN_BUTTON)){
			robot.armController.setSetpoint(robot.armAngle.pidGet() - ArmController.VOLTS_PER_DEGREE * 35.0);
			armMoveButtonToggle = true;
		} else if (armMoveButtonToggle) {
			robot.armController.setSetpoint(robot.armAngle.pidGet());
			armMoveButtonToggle = false;
		}
	}
	
	/**
	 * Operate the shooter mechanism.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operateShooter(Robot robot)
	{
		if(robot.operatorstick.getRawButton(SHOOTER_READY_BUTTON)) {
			if (!shooterReadyButtonToggle) {
				shooterReadyButtonToggle = true;
				robot.leftShooterWheel.set(1.0);
				robot.rightShooterWheel.set(-1.0);
				robot.roller.set(0);
				robot.intake.set(DoubleSolenoid.Value.kReverse);
			}
			
			if (robot.operatorstick.getRawButton(SHOOT_BUTTON)) {
				robot.kicker.set(DoubleSolenoid.Value.kForward);
			}
			else {
				robot.kicker.set(DoubleSolenoid.Value.kReverse);
			}
		}
		else if (shooterReadyButtonToggle) {
			robot.kicker.set(DoubleSolenoid.Value.kReverse);
			robot.leftShooterWheel.set(0);
			robot.rightShooterWheel.set(0);
			shooterReadyButtonToggle = false;
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
			if(robot.operatorstick.getRawButton(CLIMBER_DEPLOY_BUTTON)){
				robot.climber.set(-0.3);
			}
			else if(robot.operatorstick.getRawButton(CLIMBER_RETRACT_BUTTON)){
				robot.climber.set(0.3);
			}
			else{
				robot.climber.set(0);
			}
		}
		
		if (robot.operatorstick.getRawButton(CLIMBER_LOCK_BUTTON) && ! climberLockButtonToggle) {
			climberLockButtonToggle = true;
			if (robot.climbLock.get() == DoubleSolenoid.Value.kReverse) {
				robot.climbLock.set(DoubleSolenoid.Value.kForward);
				robot.climber.set(0);
			}
			else {
				robot.climbLock.set(DoubleSolenoid.Value.kReverse);
			}
		}
		else if (! robot.operatorstick.getRawButton(CLIMBER_LOCK_BUTTON)) {
			climberLockButtonToggle = false;
		}
		
		// TODO: the climber lock needs to be tied to the dashboard timer also, so that it auto locks just before the match ends.
	}
	
	public static void calibrateMotor(Robot robot)
	{
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
	}
}
