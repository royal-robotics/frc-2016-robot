package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class OperatorController {
	static Robot robot;
	static Joystick controller;
	
	public static void startOperator(Robot robot){
		OperatorController.robot=robot;
		OperatorController.controller=robot.operatorstick;
		pickup();
		armMotor();
		launch();
		climber();
	}
	
	private static void pickup(){
		DoubleSolenoid intake = robot.intake;
		VictorSP roller=robot.roller;
		if(controller.getRawButton(5)){
			intake.set(DoubleSolenoid.Value.kForward);
			roller.set(.5);
		}else{
			intake.set(DoubleSolenoid.Value.kReverse);
			roller.set(0);
		}
	}
	
	private static void armMotor(){
		VictorSP armMotor=robot.armMotor;
		if(controller.getRawButton(4)){
			armMotor.set(.50);
		}
		else if(controller.getRawButton(2)){
			armMotor.set(-.15);
		}
		else{
			armMotor.set(0);
		}
	}
	
	private static void launch(){
		CANTalon rightShooterWheel=robot.rightShooterWheel;
		CANTalon leftShooterWheel=robot.leftShooterWheel;
		rightShooterWheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		leftShooterWheel.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightShooterWheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
		rightShooterWheel.reverseSensor(true);
		rightShooterWheel.configNominalOutputVoltage(+0.0f, -0.0f);
		rightShooterWheel.configPeakOutputVoltage(+12.0f, -12.0f);
		rightShooterWheel.setProfile(0);
		rightShooterWheel.setF(0);
		rightShooterWheel.setP(0);
		rightShooterWheel.setI(0);
		rightShooterWheel.setD(0);
	
		DoubleSolenoid kicker = robot.kicker;

		if(controller.getRawButton(6)){
			rightShooterWheel.set(-1);
			leftShooterWheel.set(1);
			if (controller.getRawButton(8)){
				kicker.set(DoubleSolenoid.Value.kForward);
			}else{
				kicker.set(DoubleSolenoid.Value.kReverse);
			}
		}
		else{
			kicker.set(DoubleSolenoid.Value.kReverse);
			rightShooterWheel.set(0);
			leftShooterWheel.set(rightShooterWheel.getDeviceID());
		}
	}
	
	static boolean climberLocked = false;
	static boolean climbToggle = false;
	private static void climber(){
		VictorSP climber=robot.climber;
		if(controller.getRawButton(1)){
			climber.set(.4);
		}
		else if(controller.getRawButton(3)){
			climber.set(-.4);
		}
		else{
			climber.set(0);
		}
		DoubleSolenoid climbLock = robot.climbLock;
		if (controller.getRawButton(11)){
			if (!climbToggle){
				climberLocked = !climberLocked;
	    	}
			climbToggle = true;
	    }
			else{
				climbToggle = false;
			}
		if (climberLocked){
			climbLock.set(DoubleSolenoid.Value.kForward);
		}
		else{
			climbLock.set(DoubleSolenoid.Value.kReverse);
		}
	}
}
