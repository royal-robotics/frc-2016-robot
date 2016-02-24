
package org.usfirst.frc.team2522.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.GeometricAdvancedSetupDataOption;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

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
	public final int CALIBRATE_MOTOR_BUTTON = 10;
	
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
	//public static double driveTranDistancePerPulse = 1 / (256.0 * (12.0 / 36.0) * (60.0 / 24.0) * 6.0 * 3.1415);
	public static final double driveTranDistancePerPulse = (18.84/7675.0) * 4;
	//measured distance per pulse
	// <Pulses Per Rotation> * <Down Gearing> * <Wheel Diameter in Inches> * <Pi>
	public static final double climberTranDistancePerPulse = 1 / (360.0 * (42.0 / 60.0) * 1.375 * 3.1415);
	
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
	boolean shiftToggle = false;
	boolean lowGear = true;
	
	//Camera
	CameraServer camera;
	int session;
    Image frame;
	Image binaryFrame;
	int imaqError;
	NIVision.Range REFLECTIVE_RED_RANGE = new NIVision.Range(40, 75);
	NIVision.Range REFLECTIVE_GREEN_RANGE = new NIVision.Range(50, 255);
	NIVision.Range REFLECTIVE_BLUE_RANGE = new NIVision.Range(40, 75);
	
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	
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
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();
        climberEncoder.reset();
        leftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
        rightDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
        climberEncoder.setDistancePerPulse(climberTranDistancePerPulse);

        // Initialize CAN Shooter Motor Controllers
        leftShooterWheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftShooterWheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        leftShooterWheel.reverseSensor(true);
        leftShooterWheel.configNominalOutputVoltage(+0.0f, -0.0f);
        leftShooterWheel.configPeakOutputVoltage(+12.0f, -12.0f);
        leftShooterWheel.setProfile(0);
        leftShooterWheel.setF(0.1);			// TODO: calibrate the sensor to determine proper feed rate for RPM mapping
        leftShooterWheel.setP(0);
        leftShooterWheel.setI(0);
        leftShooterWheel.setD(0);

        rightShooterWheel.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightShooterWheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        rightShooterWheel.reverseSensor(false);
        rightShooterWheel.configNominalOutputVoltage(+0.0f, -0.0f);
        rightShooterWheel.configPeakOutputVoltage(+12.0f, -12.0f);
        rightShooterWheel.setProfile(0);
        rightShooterWheel.setF(0.1);			// TODO: calibrate the sensor to determine proper feed rate for RPM mapping
        rightShooterWheel.setP(0);
        rightShooterWheel.setI(0);
        rightShooterWheel.setD(0);
		
        //leftShooterWheel.reverseSensor(false);
		//leftShooterWheel.changeControlMode(CANTalon.TalonControlMode.Follower);
		//leftShooterWheel.set(rightShooterWheel.getDeviceID());	// This tells the right motor controller to follow the left one.

        
        // Initialize drive //
    	myDrive = new RobotDrive(leftDrive, rightDrive);
        myDrive.setExpiration(0.1);
        
        // Initialize camera
        //camera = CameraServer.getInstance();
        //camera.startAutomaticCapture("cam0");
        
        // create images
        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		
		//criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);

        
        // the camera name (ex "cam0") can be found through the roborio web interface
        try {
    		session = NIVision.IMAQdxOpenCamera("cam0",
                    NIVision.IMAQdxCameraControlMode.CameraControlModeController);
            NIVision.IMAQdxConfigureGrab(session);
            camera = CameraServer.getInstance();
        } catch (Exception e) {
        	System.out.println(e.toString());
        }

        SmartDashboard.putInt("red low", REFLECTIVE_RED_RANGE.minValue);
        SmartDashboard.putInt("red high", REFLECTIVE_RED_RANGE.maxValue);
        SmartDashboard.putInt("green low", REFLECTIVE_GREEN_RANGE.minValue);
        SmartDashboard.putInt("green high", REFLECTIVE_GREEN_RANGE.maxValue);
        SmartDashboard.putInt("blue low", REFLECTIVE_BLUE_RANGE.minValue);
        SmartDashboard.putInt("blue high", REFLECTIVE_BLUE_RANGE.maxValue);
        
   
        SmartDashboard.putNumber("Calibrate Motor", -1);
	}

	/**
	 * This function is called at the beginning of the autonomous period
	 */
	public void autonomousInit()
	{
		armController.enable();
        armController.setSetpoint(armAngle.pidGet());
        mxp.reset(); 	// reset the gyro setting to zero before autonomous starts so that we have a fixed bearing.
		
        // reset encoder distances for start of autonomous.
		leftDriveEncoder.reset();	
		rightDriveEncoder.reset();
		
		/*roller.set(0.0);
		intake.set(DoubleSolenoid.Value.kReverse);
		leftShooterWheel.set(0.0);
		rightShooterWheel.set(0.0);
		armController.setTargetAngle(128.6);*/
	}
	
    /**
     * This function is called periodically during autonomous
     */
	int autoCounter = 0;
	boolean toggleArmAngle = false;
    public void autonomousPeriodic()
    {
    	SmartDashboard.putInt("Auto Count", autoCounter);
    	if(autoCounter == 0) {
    		driveForward(30.0);
    	} else if(autoCounter == 1) {
    		if(!toggleArmAngle) {
    			toggleArmAngle = true;
    			armController.setTargetAngle(0.0);
    		}
    		waitArmValue(0.0);
    	} else if(autoCounter == 2) {
    		driveForward(30.0);
    	}
    	updateDashboard();

    }
    private void driveForward (double distance) {
    	if ((leftDriveEncoder.getDistance() < distance) ||  (-rightDriveEncoder.getDistance() < distance)) {
    		if(Math.abs(leftDriveEncoder.getDistance() - -rightDriveEncoder.getDistance()) < 1.0) {
    			leftDrive.set(-0.4);
    			rightDrive.set(0.4);
    		} else if(leftDriveEncoder.getDistance() > -rightDriveEncoder.getDistance()) {
    			leftDrive.set(-0.35);
    			rightDrive.set(0.4);
    		} else {
    			leftDrive.set(-0.4);
    			rightDrive.set(0.35);
    		}
    	} else {
    		leftDrive.set(0.0);
    		rightDrive.set(0.0);
    		leftDriveEncoder.reset();
    		rightDriveEncoder.reset();
    		autoCounter++;
    	}
	}
    private void waitArmValue(double angle){
    	if(Math.abs(armController.getAngle() - angle) < 5.0) {
    		toggleArmAngle = false;
    		autoCounter++;
    	}
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
    	if (rightstick.getRawButton(1) || (leftstick.getRawButton(1)))
    	{
    		//shifter.set(DoubleSolenoid.Value.kForward);
    		if (!shiftToggle){
    			lowGear = !lowGear;
    		}
    	shiftToggle = true;
    	}
    	else
    	{
    		//shifter.set(DoubleSolenoid.Value.kReverse);
    		shiftToggle = false;
    	}
    	if (lowGear){
    		shifter.set(DoubleSolenoid.Value.kForward);
    	}
    	else{
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
    	
    	// If Motor Calibration button is held, activate calibration routine
    	//
    	if (operatorstick.getRawButton(CALIBRATE_MOTOR_BUTTON)) {
    		OperatorController.calibrateMotor(this);
    	}
    	
    	if(camera != null) {
    		// Should onlyw do this when button pushed and it needs to
    		// feed the watch dog during processing.
    		//update
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
    	if(camera != null) {
    		updateCamera();
    	}
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
    	
    	SmartDashboard.putNumber("Calibrate Speed", operatorstick.getRawAxis(3));
    }
    
    public void updateCamera() {
    	NIVision.IMAQdxGrab(session, frame, 1);
        /*NIVision.imaqDrawShapeOnImage(frame, frame, rect,
                DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);*/
        //NIVision.ParticleFilterCriteria2
    	
    	NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.RGB, REFLECTIVE_RED_RANGE, REFLECTIVE_GREEN_RANGE, REFLECTIVE_BLUE_RANGE);
    	int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
    	
    	REFLECTIVE_RED_RANGE = new NIVision.Range(SmartDashboard.getInt("red low"), SmartDashboard.getInt("red high"));
    	REFLECTIVE_GREEN_RANGE = new NIVision.Range(SmartDashboard.getInt("green low"), SmartDashboard.getInt("green high"));
    	REFLECTIVE_BLUE_RANGE = new NIVision.Range(SmartDashboard.getInt("blue low"), SmartDashboard.getInt("blue high"));
    	    	
    	if(operatorstick.getPOV(0) == 270) {
            camera.setImage(binaryFrame);
    	} else {
            camera.setImage(frame);
    	}
        
    }
}