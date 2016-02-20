
package org.usfirst.frc.team2522.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * 
 * 2016 FRC Robot
 */
public class Robot extends IterativeRobot {
	// Gyro / Accelerometer sensor //
	AHRS mxp = new AHRS(SPI.Port.kMXP);

	// Motor controllers //
	VictorSP leftDrive = new VictorSP(1);
	VictorSP rightDrive = new VictorSP(5);
	VictorSP armMotor = new VictorSP(3);
	VictorSP roller = new VictorSP(7);
	VictorSP climber = new VictorSP(6);
	
	CANTalon leftShooterWheel = new CANTalon(0);
	CANTalon rightShooterWheel = new CANTalon(1);
		
	// Analog sensors //
	AnalogInput armAngle = new AnalogInput(0);

	// DIO ports //
	LEDControllerV2 led = new LEDControllerV2(new DigitalOutput(0), new DigitalOutput(1));

	// <Pulses Per Rotation> * <Encoder Mount Up Gearing> <Third Stage Down Gearing> /  * <Wheel Diameter in Inches> * <Pi>
	public static double driveTranDistancePerPulse = 256.0 * (12.0 / 36.0) * (60.0 / 24.0) * 6.0 * 3.1415;
	
	// <Pulses Per Rotation> * <Down Gearing> * <Wheel Diameter in Inches> * <Pi>
	public static double climberTranDistancePerPulse = 360.0 * (42.0 / 60.0) * 1.375 * 3.1415;
	
	Encoder rightDriveEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3));
	Encoder leftDriveEncoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
	Encoder climberEncoder = new Encoder(new DigitalInput(6), new DigitalInput (7));

	DigitalInput shooterHome = new DigitalInput(8);	// limit switch	
	
	// Pneumatic Solenoids //
	DoubleSolenoid shifter = new DoubleSolenoid(0,7);
	DoubleSolenoid climbLock = new DoubleSolenoid(1,6);
	DoubleSolenoid kicker = new DoubleSolenoid(2,5);
	DoubleSolenoid intake = new DoubleSolenoid(4, 3);
		
	// Joysticks //
	Joystick leftstick = new Joystick(0);
	Joystick rightstick = new Joystick(1);
	Joystick operatorstick = new Joystick(2);
	
	// PID Controllers
	ArmController armController;
	
	// Drive //
	RobotDrive myDrive;
	boolean driveModeToggle = false;
	boolean arcadeMode = false;
	
	/**
	 *	This function is called when the robot code is first launched 
	 */
	public void robotInit()
	{
        // Initialize Arm PID Controller
        armController = new ArmController(armAngle, armMotor);
        armController.setSetpoint(armAngle.pidGet());

        // Initialize pneumatic solenoids in their default positions;
        shifter.set(DoubleSolenoid.Value.kReverse);
        intake.set(DoubleSolenoid.Value.kReverse);
        kicker.set(DoubleSolenoid.Value.kReverse);
        climbLock.set(DoubleSolenoid.Value.kReverse);    
    	
        // Initialize Encoder Distances
        leftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
        rightDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
        climberEncoder.setDistancePerPulse(climberTranDistancePerPulse);

        // Initialize CAN Shooter Motor Controllers
        rightShooterWheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightShooterWheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        rightShooterWheel.reverseSensor(true);
        rightShooterWheel.configNominalOutputVoltage(+0.0f, -0.0f);
        rightShooterWheel.configPeakOutputVoltage(+12.0f, -12.0f);
        rightShooterWheel.setProfile(0);
        rightShooterWheel.setF(0.1);			// TODO: calibrate the sensor to determine proper feed rate for RPM mapping
        rightShooterWheel.setP(0);
        rightShooterWheel.setI(0);
        rightShooterWheel.setD(0);
		
		leftShooterWheel.reverseSensor(false);
		leftShooterWheel.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftShooterWheel.set(rightShooterWheel.getDeviceID());	// This tells the right motor controller to follow the left one.

        
        // Initialize drive //
    	myDrive = new RobotDrive(leftDrive, rightDrive);
        myDrive.setExpiration(0.1);
    }

	/**
	 * This function is called at the beginning of the autonomous period
	 */
	public void autonomousInit()
	{
        armController.enable();
        armController.setSetpoint(armAngle.pidGet());
        mxp.reset(); 	// reset the gyro setting to zero before autonomous starts so that we have a fixed bearing.
		
		leftDriveEncoder.reset();	// reset encoder distances for start of autonomous.
		rightDriveEncoder.reset(); 
	}
	
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
    	updateDashboard();
    }

    /**
	 * This function is called at the beginning of the teleop period
     */
    public void teleopInit()
    {
        armController.enable();
        armController.setSetpoint(armAngle.pidGet());
    }
    
    
    /**
     * This function is called periodically during teleop
     */
    public void teleopPeriodic()
    {
    	if (shooterHome.get()) {
//    		armController.setHomeVoltage(armAngle.pidGet());
    	}
    	
    	OperatorController.operateArm(this);
    	
    	OperatorController.operatePickup(this);
    	
    	OperatorController.operateShooter(this);
    	
    	OperatorController.operateClimber(this);
    	
    	
    	// Check for and toggle the drive mode from tank to arcade if button 3 on left stick is pressed.
    	//
    	if (leftstick.getRawButton(3) && !driveModeToggle) {
    		if (!driveModeToggle) {
    			arcadeMode = ! arcadeMode;
        		driveModeToggle = true;
    		}
    	}
    	else {
    		driveModeToggle = false;
    	}
    	
    	// Shift to high gear if either stick button 1 is pressed.
    	//
    	if (leftstick.getRawButton(1) || rightstick.getRawButton(1))
    	{
    		shifter.set(DoubleSolenoid.Value.kForward);
    	}
    	else
    	{
    		shifter.set(DoubleSolenoid.Value.kReverse);
    	}

    	// Drive the robot
    	//
    	if (arcadeMode) {
    		myDrive.arcadeDrive(leftstick);
    	}
    	else {
    		myDrive.tankDrive(leftstick, rightstick);
    	}    	
    	
    	updateDashboard();
    }

    /**
     * 
     */
    public void disabledInit()
    {
    	armController.disable();
    	updateDashboard();
    }

    /**
     * 
     */
    public void disabledPeriodic()
    {
    	updateDashboard();
    }
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
    	updateDashboard();
    }
    
    /**************************************************************************************
     * Helper methods for the robot.
     */
    
    /**
     * All dashboard updating should be done here, so that the values are updated in all robot operation modes.
     */
    public void updateDashboard()
    {
    	SmartDashboard.putNumber("Gyro", mxp.getAngle());
    	SmartDashboard.putNumber("Velocity f/s", mxp.getVelocityX() * 0.3048 /*(f/M)*/);

    	SmartDashboard.putString("Drive Gear", (shifter.get() == DoubleSolenoid.Value.kReverse) ? "Low" : "High");
    	SmartDashboard.putString("Drive Mode", arcadeMode ? "Arcade" : "Tank");
    	SmartDashboard.putNumber("LE Raw", leftDriveEncoder.getRaw());
    	SmartDashboard.putNumber("RE Raw", rightDriveEncoder.getRaw());
    	SmartDashboard.putNumber("LE Inches.", leftDriveEncoder.getDistance());
    	SmartDashboard.putNumber("RE Inches", rightDriveEncoder.getDistance());
    	

    	SmartDashboard.putNumber("LS Motor", leftShooterWheel.get());
    	SmartDashboard.putNumber("RS Motor", rightShooterWheel.get());
    	SmartDashboard.putNumber("LS Speed", leftShooterWheel.getSpeed());
    	SmartDashboard.putNumber("RS Speed", rightShooterWheel.getSpeed());
    	SmartDashboard.putNumber("LS Error", leftShooterWheel.getClosedLoopError());
    	SmartDashboard.putNumber("RS Error", rightShooterWheel.getClosedLoopError());
    	
    	SmartDashboard.putNumber("Arm Volts", armAngle.pidGet());	// This is the value used for PID Input.
    	SmartDashboard.putNumber("Arm Angle", armController.getAngle());

    	SmartDashboard.putNumber("Arm Motor", armMotor.get());
    	SmartDashboard.putNumber("Arm Ctrl", armController.get());
    	SmartDashboard.putNumber("Arm Error", armController.getAvgError());
    	SmartDashboard.putNumber("Arm Target", armController.getSetpoint());
    	
    	SmartDashboard.putNumber("Arm Staight Volts", armController.fullyExtendedVoltage);
    	SmartDashboard.putNumber("Arm Home Volts", armController.homeVoltage);
    	SmartDashboard.putNumber("Arm Floor Volts", armController.floorVoltage);
    	SmartDashboard.putNumber("Arm Home Angle", (armController.homeVoltage - armController.fullyExtendedVoltage) / ArmController.voltsPerDegree);
    }
    
}