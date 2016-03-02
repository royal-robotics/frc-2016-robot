
package org.usfirst.frc.team2522.robot;

import java.util.Comparator;
import java.util.Vector;

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
	//A structure to hold measurements of a particle
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
		
		public int compareTo(ParticleReport r)
		{
			return (int)(r.Area - this.Area);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int)(r1.Area - r2.Area);
		}
	};

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
	boolean driveStraightToggle = false;
	
	//Camera
	CameraServer camera;
	int session = -1;
    Image frame;
	Image binaryFrame;
	int imaqError;
	NIVision.Range REFLECTIVE_RED_RANGE = new NIVision.Range(20, 70);
	NIVision.Range REFLECTIVE_GREEN_RANGE = new NIVision.Range(75, 255);
	NIVision.Range REFLECTIVE_BLUE_RANGE = new NIVision.Range(20, 90);
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
        shifter.set(DoubleSolenoid.Value.kForward);	// low gear
        intake.set(DoubleSolenoid.Value.kReverse);
        kicker.set(DoubleSolenoid.Value.kReverse);
        climbLock.set(DoubleSolenoid.Value.kReverse);    

        // Initialize Encoder Distances
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();
        climberEncoder.reset();
        leftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
        rightDriveEncoder.setDistancePerPulse(-driveTranDistancePerPulse);
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
        myDrive.setExpiration(0.5);	// this allows camera code enough time to execute between loops.
        
        // Initialize camera
        //
        try
        {
            // The camera name (ex "cam0") can be found through the roborio web interface
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
    		
    		// Setup image processing criteria
    		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, 0.5, 100.0, 0, 0);
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
        
        // This value is used to calibrate motor controllers
        // 
        SmartDashboard.putNumber("Calibrate Motor", -1);
        
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
		
		armController.enable();
        armController.setSetpoint(armAngle.pidGet());
        
		// we should start in low gear
		shifter.set(DoubleSolenoid.Value.kForward);
		
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
        armController.enable();
        armController.setSetpoint(armAngle.pidGet());

        shifter.set(DoubleSolenoid.Value.kForward); // low gear
        
        leftShooterWheel.set(0.0);
        rightShooterWheel.set(0.0);
		kicker.set(DoubleSolenoid.Value.kReverse);

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
    	
		OperatorController.calibrateMotor(this); // If Motor Calibration button is held, activate calibration routine
    	
    	
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
    	
    	// Toggle shifter if either stick button 1 is pressed.
    	//
    	if (rightstick.getRawButton(1) || (leftstick.getRawButton(1)))
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
    	if (rightstick.getRawButton(4))
    	{
    		if (!driveStraightToggle)
    		{
    			AutonomousController.driveResetBearing(this);
    			driveStraightToggle = true;
    		}
    		AutonomousController.driveResetEncoders(this);
    		AutonomousController.driveForward(this, 0, 0.80, 15.0);
    	}
    	else
    	{
			driveStraightToggle = false;
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
     * All dashboard updating should be done here, so that the values are updated in all robot operation modes.
     */
    public void updateDashboard()
    {
    	SmartDashboard.putNumber("Gyro Angle", mxp.getAngle());
    	SmartDashboard.putNumber("Compass Heading", mxp.getCompassHeading());
    	SmartDashboard.putNumber("Velocity f/s", mxp.getVelocityX() * 0.3048 /*(f/M)*/);

    	SmartDashboard.putString("Auto Mode", AutonomousController.autoModeString);
    	SmartDashboard.putNumber("Auto Step", AutonomousController.autoStep);
    	
    	SmartDashboard.putString("Drive Gear", (shifter.get() == DoubleSolenoid.Value.kForward) ? "Low" : "High");
    	SmartDashboard.putString("Drive Mode", arcadeMode ? "Arcade" : "Tank");
    	SmartDashboard.putNumber("LE Raw", leftDriveEncoder.getRaw());
    	SmartDashboard.putNumber("RE Raw", rightDriveEncoder.getRaw());
    	SmartDashboard.putNumber("LE Inches.", leftDriveEncoder.getDistance());
    	SmartDashboard.putNumber("RE Inches", rightDriveEncoder.getDistance());
    	
    	SmartDashboard.putNumber("CLM Raw", climberEncoder.getRaw());
    	SmartDashboard.putNumber("CLM Inches.", climberEncoder.getDistance());
    	
    	SmartDashboard.putNumber("LS Motor", leftShooterWheel.get());
    	SmartDashboard.putNumber("RS Motor", rightShooterWheel.get());
    	SmartDashboard.putNumber("LS Speed", leftShooterWheel.getSpeed());
    	SmartDashboard.putNumber("RS Speed", rightShooterWheel.getSpeed());
    	SmartDashboard.putNumber("LS Error", leftShooterWheel.getClosedLoopError());
    	SmartDashboard.putNumber("RS Error", rightShooterWheel.getClosedLoopError());
    	
    	SmartDashboard.putNumber("Arm Volts", armAngle.pidGet());	// This is the value used for PID Input.
    	SmartDashboard.putNumber("Arm Angle", armController.getAngle());
    	SmartDashboard.putBoolean("Arm Home", shooterHomeSwitch.get());

    	SmartDashboard.putNumber("Arm Motor", armMotor.get());
    	SmartDashboard.putNumber("Arm Ctrl", armController.get());
    	SmartDashboard.putNumber("Arm Error", armController.getAvgError());
    	SmartDashboard.putNumber("Arm Target", armController.getSetpoint());
    	SmartDashboard.putBoolean("Arm On Target", armController.onTarget());
    	
    	SmartDashboard.putNumber("Arm Staight Volts", armController.fullyExtendedVoltage);
    	SmartDashboard.putNumber("Arm Home Volts", armController.homeVoltage);
    	SmartDashboard.putNumber("Arm Floor Volts", armController.floorVoltage);
    	SmartDashboard.putNumber("Arm Home Angle", (armController.homeVoltage - armController.fullyExtendedVoltage) / ArmController.VOLTS_PER_DEGREE);
    	
    	SmartDashboard.putBoolean("Ball Loaded", ballLoaded);
    	SmartDashboard.putBoolean("Ball Load Switch", ballLoadedSwitch.get());
    	
    	SmartDashboard.putNumber("Calibrate Speed", operatorstick.getRawAxis(3));
    }
    
    public void updateCamera()
    {
    	if (camera != null)
    	{
	    	NIVision.IMAQdxGrab(session, frame, 1);	// grab the raw image frame from the camera
	    	
    		if (operatorstick.getPOV(0) == 270)
    		{
		    	REFLECTIVE_RED_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("red low"), (int)SmartDashboard.getNumber("red high"));
		    	REFLECTIVE_GREEN_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("green low"), (int)SmartDashboard.getNumber("green high"));
		    	REFLECTIVE_BLUE_RANGE = new NIVision.Range((int)SmartDashboard.getNumber("blue low"), (int)SmartDashboard.getNumber("blue high"));
		    	
		    	NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.RGB, REFLECTIVE_RED_RANGE, REFLECTIVE_GREEN_RANGE, REFLECTIVE_BLUE_RANGE);

		    	camera.setImage(binaryFrame);

		    	//filter out small particles
				float areaMin = (float)SmartDashboard.getNumber("Area min %", 0.5);
				criteria[0].lower = areaMin;
				imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);
				int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Masked particles", numParticles);

				if(numParticles > 0)
				{
					//Measure particles and sort by particle size
					Vector<ParticleReport> particles = new Vector<ParticleReport>();
					for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
					{
						ParticleReport par = new ParticleReport();
						par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
						par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
						par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
						par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
						par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
						par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
						particles.add(par);
					}
					particles.sort(null);

					//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
					//for the reader. Note that this scores and reports information about a single particle (single L shaped target). To get accurate information 
					//about the location of the tote (not just the distance) you will need to correlate two adjacent targets in order to find the true center of the tote.
					//scores.Aspect = AspectScore(particles.elementAt(0));
					//SmartDashboard.putNumber("Aspect", scores.Aspect);
					//scores.Area = AreaScore(particles.elementAt(0));
					//SmartDashboard.putNumber("Area", scores.Area);
					//boolean isTote = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;

					//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
					//SmartDashboard.putBoolean("IsTote", isTote);
					//SmartDashboard.putNumber("Distance", computeDistance(binaryFrame, particles.elementAt(0)));
				}
				
		    	//camera.setImage(binaryFrame);
			}
    		else {
		    	//NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
		        /*NIVision.imaqDrawShapeOnImage(frame, frame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);*/
	            camera.setImage(frame);
    		}
    	}
    }
}