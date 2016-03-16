
package org.usfirst.frc.team2522.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;

import edu.wpi.first.wpilibj.*;
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
public class Robot extends IterativeRobot
{
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
	AnalogInput frontSonicRange = new AnalogInput(1);
	AnalogInput rearSonicRange = new AnalogInput(2);
	

	// DIO ports //
	LEDControllerV2 led = new LEDControllerV2(new DigitalOutput(0), new DigitalOutput(1));

	// (<Wheel Diameter in Inches> * <Pi>) / (<Pulses Per Rotation> * <Encoder Mount Gearing> <Third Stage Gearing>)
	public static double driveTranDistancePerPulse = (6.0 * 3.1415) / (256.0 * (36.0 / 12.0) * (60.0 / 24.0));

	// (<Wheel Diameter in Inches> * <Pi>) / (<Pulses Per Rotation> * <Output Shaft Gearing>)
	public static final double climberTranDistancePerPulse = (1.375 * 3.1415) / (360.0 * (60.0 / 44.0));
	
	Encoder rightDriveEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3));
	Encoder leftDriveEncoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
	Encoder climberEncoder = new Encoder(new DigitalInput(6), new DigitalInput (7));

	DigitalInput shooterHomeSwitch = new DigitalInput(8);	// limit switch	indicating arm home position
	DigitalInput ballLoadedSwitch = new DigitalInput(9);  // limit switch indicating ball loaded.
	
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
	boolean ballLoaded = false;
	
	// Drive //
	RobotDrive myDrive;
	boolean driveModeToggle = false;
	boolean arcadeMode = false;
	boolean shiftToggle = false;
	double trackingAngle = 180.0;
	boolean driveStraightToggle = false;
	boolean turnAroundToggle = false;
	
	//Camera
	CameraServer camera;
	int session = -1;
    Image frame;
	Image binaryFrame;
	Image particalFrame;
	NIVision.Range REFLECTIVE_RED_RANGE = new NIVision.Range(0, 180);
	NIVision.Range REFLECTIVE_GREEN_RANGE = new NIVision.Range(220, 255);
	NIVision.Range REFLECTIVE_BLUE_RANGE = new NIVision.Range(0, 255);
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	
	boolean useRPMs = true;
	
	/**
	 *	This function is called when the robot code is first launched 
	 */
	public void robotInit()
	{
        // Initialize Arm PID Controller
        armController = new ArmController(armAngle, armMotor);
        armController.setSetpoint(armAngle.pidGet());

        // Initialize pneumatic solenoids in their default positions;
        shifter.set(DoubleSolenoid.Value.kForward);	// low gear
        intake.set(DoubleSolenoid.Value.kReverse);
        kicker.set(DoubleSolenoid.Value.kReverse);
        climbLock.set(DoubleSolenoid.Value.kReverse);    

        // Initialize Encoder Distances
        leftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
        leftDriveEncoder.reset();
        
        rightDriveEncoder.setDistancePerPulse(-driveTranDistancePerPulse);
        rightDriveEncoder.reset();

        climberEncoder.setDistancePerPulse(climberTranDistancePerPulse);
        climberEncoder.reset();

        // Initialize CAN Shooter Motor Controllers
        if (this.useRPMs)
        {
	        leftShooterWheel.changeControlMode(CANTalon.TalonControlMode.Speed);
	        rightShooterWheel.changeControlMode(CANTalon.TalonControlMode.Speed);
        }
        else
        {
	        leftShooterWheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	
	        rightShooterWheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        }
        
        leftShooterWheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        leftShooterWheel.reverseSensor(true);
        leftShooterWheel.configNominalOutputVoltage(+0.0f, -0.0f);
        leftShooterWheel.configPeakOutputVoltage(+12.0f, -12.0f);
        leftShooterWheel.setProfile(0);
        leftShooterWheel.setF(0.028098);			// TODO: calibrate the sensor to determine proper feed rate for RPM mapping
        leftShooterWheel.setP(0);
        leftShooterWheel.setI(0);
        leftShooterWheel.setD(0);

        rightShooterWheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        rightShooterWheel.reverseSensor(false);
        rightShooterWheel.configNominalOutputVoltage(+0.0f, -0.0f);
        rightShooterWheel.configPeakOutputVoltage(+12.0f, -12.0f);
        rightShooterWheel.setProfile(0);
        rightShooterWheel.setF(0.032948);		// TODO: calibrate the sensor to determine proper feed rate for RPM mapping
        rightShooterWheel.setP(0);
        rightShooterWheel.setI(0);
        rightShooterWheel.setD(0);
		
        //leftShooterWheel.reverseSensor(false);
		//leftShooterWheel.changeControlMode(CANTalon.TalonControlMode.Follower);
		//leftShooterWheel.set(rightShooterWheel.getDeviceID());	// This tells the right motor controller to follow the left one.

        
        // Initialize drive //
    	myDrive = new RobotDrive(leftDrive, rightDrive);

    	// set motor timeouts high enough that camera vision processing will not interfere.
    	double motorTimeOut = 0.5;
    	myDrive.setExpiration(motorTimeOut);
    	armMotor.setExpiration(motorTimeOut);
    	rightShooterWheel.setExpiration(motorTimeOut);
    	leftShooterWheel.setExpiration(motorTimeOut);
    	roller.setExpiration(motorTimeOut);
    	climber.setExpiration(motorTimeOut);        
        
        // Initialize camera
        //
        try
        {
            // The camera name (ex "cam1") can be found through the roborio web interface http://roboRIO-2522-frc.local/
        	//
        	// This is used for normal camera image updating to dashboard, but not used when doing image processing
        	//
            //camera.startAutomaticCapture("cam0");
            //camera = CameraServer.getInstance();
        	
    		session = NIVision.IMAQdxOpenCamera("cam1", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
            NIVision.IMAQdxConfigureGrab(session);
            camera = CameraServer.getInstance();
            
            // Create image objects
            frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
    		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
    		particalFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
    		
    		// Setup image processing criteria
    		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, 0.05, 100.0, 0, 0);
        }
        catch (Exception e)
        {
        	System.out.println(e.toString());
        }

        SmartDashboard.putNumber("red low", REFLECTIVE_RED_RANGE.minValue);
        SmartDashboard.putNumber("red high", REFLECTIVE_RED_RANGE.maxValue);
        SmartDashboard.putNumber("green low", REFLECTIVE_GREEN_RANGE.minValue);
        SmartDashboard.putNumber("green high", REFLECTIVE_GREEN_RANGE.maxValue);
        SmartDashboard.putNumber("blue low", REFLECTIVE_BLUE_RANGE.minValue);
        SmartDashboard.putNumber("blue high", REFLECTIVE_BLUE_RANGE.maxValue);

        SmartDashboard.putNumber("Target Area Min %", 0.05);
        SmartDashboard.putNumber("Target RPM", 2500.0);
        SmartDashboard.putNumber("Target PWR", 0.5);
        
        updateDashboard();
	}

	/**
	 * This function is called at the beginning of the autonomous period
	 */
	public void autonomousInit()
	{
        // reset the gyro setting to zero before autonomous starts so that we have a fixed bearing.
        AutonomousController.driveResetBearing(this);
        
        // reset encoder distances for start of autonomous.
        AutonomousController.driveResetEncoders(this);
		
		// enable arm controller
		armController.enable();
        armController.setSetpoint(armAngle.pidGet());
        
		// we should start in low gear
		shifter.set(DoubleSolenoid.Value.kForward);	// low gear
		
		// kicker should be retracted
		kicker.set(DoubleSolenoid.Value.kReverse);
		
		// intake roller should be stopped and pulled back.
		roller.set(0.0);
		intake.set(DoubleSolenoid.Value.kReverse);
		
		// shooter wheels should not be moving
		leftShooterWheel.set(0.0);
		rightShooterWheel.set(0.0);
		
		// This is the last time getAutoMode() should be called during the autonomous phase.
		AutonomousController.getAutoMode(this);
		AutonomousController.autoStep = 0;
		
		ballLoaded = true;	// we always start autonomous with a ball loaded.
		
        updateDashboard();
	}
	
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
    	AutonomousController.RunAutonomous(this);
    	updateDashboard();
    }
    
    
    /**
	 * This function is called at the beginning of the teleop period
     */
    public void teleopInit()
    {
		// enable arm controller
		armController.enable();
        armController.setSetpoint(armAngle.pidGet());
        
		// we should start in low gear
		shifter.set(DoubleSolenoid.Value.kForward);	// low gear

		// kicker should be retracted
		kicker.set(DoubleSolenoid.Value.kReverse);
		
		// intake roller should be stopped and pulled back.
		roller.set(0.0);
		intake.set(DoubleSolenoid.Value.kReverse);
		
		// shooter wheels should not be moving
		leftShooterWheel.set(0.0);
		rightShooterWheel.set(0.0);
		
        updateDashboard();
    }
    
    
    /**
     * This function is called periodically during teleop
     */
    public void teleopPeriodic()
    {
    	OperatorController.operateArm(this);
    	
    	OperatorController.operatePickup(this);
    	
    	OperatorController.operateShooter(this);
    	
    	OperatorController.operateClimber(this);
    	
    	// Check for and toggle the drive mode from tank to arcade if button 3 on left stick is pressed.
    	//
    	if (leftstick.getRawButton(OperatorController.DRIVE_MODE_CHANGE_BUTTON) && !driveModeToggle) {
    		if (!driveModeToggle) {
    			arcadeMode = ! arcadeMode;
        		driveModeToggle = true;
    		}
    	}
    	else {
    		driveModeToggle = false;
    	}
    	
    	// Toggle shifter if either stick button 1 is pressed.
    	//
    	if (rightstick.getRawButton(OperatorController.TOGGLE_SHIFTER_BUTTON) || (leftstick.getRawButton(OperatorController.TOGGLE_SHIFTER_BUTTON)))
    	{
    		if (!shiftToggle){
    			if (shifter.get() == DoubleSolenoid.Value.kForward) {
    				shifter.set(DoubleSolenoid.Value.kReverse); // high gear
    			}
    			else {
    				shifter.set(DoubleSolenoid.Value.kForward); // low gear
    			}
    		}
    		shiftToggle = true;
    	}
    	else
    	{
    		shiftToggle = false;
    	}
    	
    	// Drive the robot
    	//
    	if (leftstick.getRawButton(OperatorController.TRACK_TARGET_BUTTON) || rightstick.getRawButton(OperatorController.TRACK_TARGET_BUTTON))
    	{
    		if (trackingAngle == 180.0)
    		{
    			trackingAngle = AutonomousController.driveGetBearing(this) - AutonomousController.getTrackingAngle(this);
    		}
    		else
    		{
    			if (AutonomousController.drivePivot(this, trackingAngle, 0.50))
    			{
    				// this will cause the target image to update when we reach the target rotation
    				AutonomousController.getTarget(this);
    			}
    		}
    	}
    	else if (rightstick.getRawButton(OperatorController.DRIVE_STRAIGHT_BUTTON))
    	{
    		if (!driveStraightToggle)
    		{
    			AutonomousController.driveResetBearing(this);
    			driveStraightToggle = true;
    		}
    		AutonomousController.driveResetEncoders(this);
    		AutonomousController.driveForward(this, 0, 0.80, 15.0);
    	}
    	else if (rightstick.getRawButton(OperatorController.TURN_AROUND_BUTTON))
    	{
    		if (!turnAroundToggle)
    		{
    			AutonomousController.driveResetBearing(this);
    			turnAroundToggle = true;
    		}
    		AutonomousController.driveTurnAround(this, 0.80);
    	}
    	else
    	{
			trackingAngle = 180.0;
			driveStraightToggle = false;
			turnAroundToggle = false;
			
	    	if (arcadeMode) {
	    		myDrive.arcadeDrive(leftstick);
	    	}
	    	else {
	    		myDrive.tankDrive(leftstick, rightstick, true);
	    	}
    	}
    	
		updateCamera();
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
    	// Cause the autonomous mode value to get updated from state of controllers.
		AutonomousController.getAutoMode(this);

    	updateCamera();
    	updateDashboard();
    }
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
		updateCamera();
    	updateDashboard();
    }
    
    /**************************************************************************************
     * Helper methods for the robot.
     */
    
    /**
     * 
     * @return	The front range in inches from the sonic range sensor.
     */
    public double getFrontRange()
    {
    	if (this.frontSonicRange != null)
    	{
    		return this.frontSonicRange.getVoltage() * 100.0;
    	}
    	
    	return -1.0;
    }

    /**
     * 
     * @return	The rear range in inches from the sonic range sensor.
     */
    public double getRearRange()
    {
    	if (rearSonicRange != null)
    	{
    		return this.rearSonicRange.getVoltage() * 100.0;
    	}
    	
    	return -1.0;
    }
    
    /**
     * All dashboard updating should be done here, so that the values are updated in all robot operation modes.
     */
    public void updateDashboard()
    {
    	SmartDashboard.putNumber("Gyro Angle", mxp.getAngle());
    	SmartDashboard.putNumber("Compass Heading", mxp.getCompassHeading());
    	SmartDashboard.putNumber("Velocity f/s", mxp.getVelocityX() * 0.3048 /*(f/M)*/);

    	SmartDashboard.putString("Auto Mode", AutonomousController.autoModeString);
    	SmartDashboard.putNumber("Auto Step", AutonomousController.autoStep);
    	
    	SmartDashboard.putString("Drive Gear", (shifter.get() == DoubleSolenoid.Value.kForward) ? "LOW" : "HIGH");
    	SmartDashboard.putString("Drive Mode", arcadeMode ? "Arcade" : "Tank");
//    	SmartDashboard.putNumber("LE Raw", leftDriveEncoder.getRaw());
//    	SmartDashboard.putNumber("RE Raw", rightDriveEncoder.getRaw());
    	SmartDashboard.putNumber("LE Inches.", leftDriveEncoder.getDistance());
    	SmartDashboard.putNumber("RE Inches", rightDriveEncoder.getDistance());
    	
    	SmartDashboard.putNumber("CLM Raw", climberEncoder.getRaw());
    	SmartDashboard.putNumber("CLM Inches.", climberEncoder.getDistance());
    	SmartDashboard.putString("CLM Lock", this.climbLock.get() == DoubleSolenoid.Value.kForward ? "LOCKED" : "UNLOCKED");
    	
//    	SmartDashboard.putBoolean("Ball Loaded", ballLoaded);
//    	SmartDashboard.putBoolean("Ball Load Switch", ballLoadedSwitch.get());
    	
    	SmartDashboard.putString("Intake:", intake.get() == DoubleSolenoid.Value.kForward ? "DOWN" : "UP");
    	
    	SmartDashboard.putNumber("LS Motor", leftShooterWheel.get());
    	SmartDashboard.putNumber("RS Motor", rightShooterWheel.get());
    	SmartDashboard.putNumber("LS Speed", leftShooterWheel.getSpeed());
    	SmartDashboard.putNumber("RS Speed", rightShooterWheel.getSpeed());
//    	SmartDashboard.putNumber("LS Error", leftShooterWheel.getClosedLoopError());
//    	SmartDashboard.putNumber("RS Error", rightShooterWheel.getClosedLoopError());
    	
    	SmartDashboard.putNumber("Front Range", this.getFrontRange());
    	SmartDashboard.putNumber("Rear Range", this.getRearRange());

    	SmartDashboard.putNumber("Arm Volts", armAngle.pidGet());	// This is the value used for PID Input.
    	SmartDashboard.putNumber("Arm Angle", armController.getAngle());
    	SmartDashboard.putNumber("Arm Motor", armMotor.get());

    	SmartDashboard.putNumber("Arm Target", armController.getSetpoint());
    	SmartDashboard.putNumber("Arm Ctrl", armController.get());
    	SmartDashboard.putNumber("Arm Error", armController.getAvgError());
//    	SmartDashboard.putBoolean("Arm On Target", armController.onTarget());

    	SmartDashboard.putNumber("Arm Staight Volts", armController.fullyExtendedVoltage);
    	SmartDashboard.putNumber("Arm Home Volts", armController.homeVoltage);
    	SmartDashboard.putNumber("Arm Floor Volts", armController.floorVoltage);
//    	SmartDashboard.putNumber("Arm Home Angle", (armController.homeVoltage - armController.fullyExtendedVoltage) / ArmController.VOLTS_PER_DEGREE);

//    	SmartDashboard.putBoolean("Arm Home", shooterHomeSwitch.get());
    }
    
    public void updateCamera()
    {
    	if (camera != null)
    	{
	    	if (leftstick.getRawButton(OperatorController.TRACK_TARGET_BUTTON) || rightstick.getRawButton(OperatorController.TRACK_TARGET_BUTTON))
	    	{
	    		camera.setImage(frame);
	    	}
	    	else if (leftstick.getRawButton(OperatorController.SHOW_TARGETS_BUTTON) || rightstick.getRawButton(OperatorController.SHOW_TARGETS_BUTTON))
    		{
	    		AutonomousController.getTrackingAngle(this);
	    		camera.setImage(frame);
			}
	    	else if (leftstick.getRawButton(OperatorController.SHOW_IMAGE_FILTER_BUTTON) || rightstick.getRawButton(OperatorController.SHOW_IMAGE_FILTER_BUTTON))
    		{
	    		AutonomousController.getTrackingAngle(this);
	    		camera.setImage(binaryFrame);
			}
	    	else
	    	{
    			NIVision.IMAQdxGrab(session, frame, 1);	// grab the raw image frame from the camera
	    		camera.setImage(frame);
	    	}
    	}
    }
}