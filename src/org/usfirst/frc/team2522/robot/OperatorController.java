package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class OperatorController
{
	public static final int climberDeployButton = 1;
	public static final int armDownButton = 2;
	public static final int climberRetractButton = 3;
	public static final int armUpButton = 4;
	public static final int pickupButton = 5;
	public static final int spitoutButton = 7;
	public static final int shooterPosButton = 9;
	public static final int shooterReadyButton = 6;
	public static final int shootButton = 8;
	public static final int climberLockButton = 11;
	
	// Used to stop the arm after arm move buttons are no longer being pressed.
	//
	static boolean armMoveButtonToggle = false;
	static boolean pickupButtonToggle = false;
	static boolean spitoutButtonToggle = false;
	static boolean shooterPosButtonToggle = false;
	static boolean shooterReadyButtonToggle = false;
	static boolean climberLockButtonToggle = false;
		
	/**
	 * Checks the robot control state and activates the pickup system accordingly.
	 * 
	 * @param robot The robot being operated.
	 */
	public static void operatePickup(Robot robot)
	{
		if (robot.operatorstick.getPOV(0) == 0){
			robot.intake.set(DoubleSolenoid.Value.kForward);
		}
		else if (robot.operatorstick.getPOV(0) == 180)
		{
			robot.intake.set(DoubleSolenoid.Value.kReverse);
		}
		
		if(robot.operatorstick.getRawButton(pickupButton) && ! robot.operatorstick.getRawButton(shooterReadyButton)) {
			if (!pickupButtonToggle)
			{
				robot.roller.set(-0.5);
				robot.intake.set(DoubleSolenoid.Value.kForward);
				robot.leftShooterWheel.set(-0.4);
				robot.rightShooterWheel.set(0.4);
				robot.armController.setTargetAngle(-15.0);
				pickupButtonToggle = true;
			}
		} else if(robot.operatorstick.getRawButton(spitoutButton) && ! robot.operatorstick.getRawButton(shooterReadyButton)) {
			if (!spitoutButtonToggle) {
				robot.roller.set(0.5);
				robot.intake.set(DoubleSolenoid.Value.kReverse);
				robot.leftShooterWheel.set(0.4);
				robot.rightShooterWheel.set(-0.4);
				robot.armController.setTargetAngle(-15);
				spitoutButtonToggle = true;
			}	
		}
		else if (pickupButtonToggle || spitoutButtonToggle) {
			robot.roller.set(0);
			
			if(robot.leftstick.getRawButton(1)) {
				robot.intake.set(DoubleSolenoid.Value.kForward);
			} else {
				robot.intake.set(DoubleSolenoid.Value.kReverse);
			}
			
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
		
		if (robot.operatorstick.getRawButton(shooterPosButton)) {
			if(!shooterPosButtonToggle) {
				robot.armController.setTargetAngle(62.0);
				shooterPosButtonToggle = true;	
			}
		}
		else if (shooterPosButtonToggle) {
			robot.armController.setSetpoint(robot.armAngle.pidGet());
			shooterPosButtonToggle = false;
		}
		
		if(robot.operatorstick.getRawButton(armUpButton)){
			robot.armController.setSetpoint(robot.armAngle.pidGet() + ArmController.voltsPerDegree * 35.0);
			armMoveButtonToggle = true;
		} else if(robot.operatorstick.getRawButton(armDownButton)){
			robot.armController.setSetpoint(robot.armAngle.pidGet() - ArmController.voltsPerDegree * 35.0);
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
		if(robot.operatorstick.getRawButton(shooterReadyButton)) {
			if (!shooterReadyButtonToggle) {
				shooterReadyButtonToggle = true;
				robot.leftShooterWheel.set(1.0);
				robot.rightShooterWheel.set(-1.0);
				robot.roller.set(0);
				robot.intake.set(DoubleSolenoid.Value.kReverse);
			}
			
			if (robot.operatorstick.getRawButton(shootButton)) {
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
			if(robot.operatorstick.getRawButton(climberDeployButton)){
				robot.climber.set(.4);
			}
			else if(robot.operatorstick.getRawButton(climberRetractButton)){
				robot.climber.set(-.4);
			}
			else{
				robot.climber.set(0);
			}
		}
		
		if (robot.operatorstick.getRawButton(climberLockButton) && ! climberLockButtonToggle) {
			climberLockButtonToggle = true;
			if (robot.climbLock.get() == DoubleSolenoid.Value.kReverse) {
				robot.climbLock.set(DoubleSolenoid.Value.kForward);
				robot.climber.set(0);
			}
			else {
				robot.climbLock.set(DoubleSolenoid.Value.kReverse);
			}
		}
		else if (! robot.operatorstick.getRawButton(climberLockButton)) {
			climberLockButtonToggle = false;
		}
		
		// TODO: the climber lock needs to be tied to the dashboard timer also, so that it auto locks just before the match ends.
	}
	
	public static void calibrateMotor(Robot robot) {
		double motorPort = SmartDashboard.getNumber("Calibrate Motor");
		VictorSP motor = null;
		boolean valid = true;
		
		switch ((int)motorPort) {
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
