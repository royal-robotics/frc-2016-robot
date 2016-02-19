
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
	
	// Motor controllers //
	VictorSP leftDrive;
	VictorSP rightDrive;
	VictorSP roller;
	VictorSP climber;
	VictorSP armMotor;
	
	CANTalon leftShooterWheel;
	CANTalon rightShooterWheel;
		
	// Pot sensor //
	AnalogInput armAngle;
	PIDController armController;
	
	// Solenoids //
	DoubleSolenoid shifter;
	DoubleSolenoid climbLock;
	DoubleSolenoid kicker;
	DoubleSolenoid intake;
	
	// Encoders //
	Encoder leftDriveEncoder;
	Encoder rightDriveEncoder;
	Encoder climberEncoder;
	
	// LED's //
	DigitalOutput ledClock;
	DigitalOutput ledData;
	LEDControllerV2 led;
	
	// Joysticks //
	Joystick leftstick;
	Joystick rightstick;
	Joystick operatorstick;
	
	// Drive //
	RobotDrive myDrive;
	
	// Gyro sensor //
	AHRS mxp;

	
    // Init toggle values
    boolean shiftToggle = false;
    boolean shiftPressed = false;

    /**
     * This function is called periodically during operator control
     */
    int tempColor = 0;
    boolean clockOn = false;
    boolean previousButtonClock = false;
    boolean dataOn = false;
    boolean previousButtonData = false;

	public void robotInit() {
		
		//leds = new LEDController(new DigitalOutput(16), new DigitalOutput(10), new DigitalOutput(11));
    	//leds = new LEDController(new DigitalOutput(16), new DigitalOutput(18), new DigitalOutput(19));
    	//do0.set(true);
    	//do1.set(true);
    	//leds.setColor(LEDUtil.Color.WHITE);

		//Init DIO ports (encoders)//
		leftDriveEncoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
		rightDriveEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3));
		climberEncoder = new Encoder(new DigitalInput(6), new DigitalInput (7));

    	
		//Init motors
		rightDrive = new VictorSP(5);
		leftDrive = new VictorSP(1);
		armMotor = new VictorSP(3);
		roller = new VictorSP(7);
		climber = new VictorSP(6);
		
		leftShooterWheel = new CANTalon(0);
		rightShooterWheel = new CANTalon(1);
		
		// Init solenoids //
		shifter = new DoubleSolenoid(0,7);
		climbLock = new DoubleSolenoid(1,6);
		kicker = new DoubleSolenoid(2,5);
		intake = new DoubleSolenoid(3,4);
		
		// Init Joysticks //
        leftstick = new Joystick(0);
        rightstick = new Joystick(1);
        operatorstick = new Joystick(2);
        
        // Init LED's //
        ledClock = new DigitalOutput(0);
        ledData = new DigitalOutput(1);
        led = new LEDControllerV2(ledClock, ledData);
    
    	
        // Init drive //
    	myDrive = new RobotDrive(leftDrive, rightDrive);
        myDrive.setExpiration(0.1);
//        myDrive.setInvertedMotor(MotorType.kFrontLeft, true);
//        myDrive.setInvertedMotor(MotorType.kFrontRight, true);
        
        // Init pot sensor //
        armAngle= new AnalogInput(0);
        
        armController = new PIDController(100.0, 0.000, 0.0, armAngle, armMotor);
        armController.setInputRange(2.267, 2.800);
        armController.setOutputRange(-1.0, 1.0);
        armController.setPercentTolerance(5.0);
        
        // Init gyro sensor //
		mxp = new AHRS(SPI.Port.kMXP);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    
    	updateDashboard();
    }

    
    public void teleopPeriodic() {
    	/*if(tempColor == 255) {
    		tempColor = 0;
    	} else {
    		tempColor++;
    	}*/
    	
    	    	
        //led.ledColor = new LEDColor(255, 0, 0);
        //led.writeColor();
    	if(rightstick.getRawButton(1) && !previousButtonData) {
    		dataOn = !dataOn;
    		if(dataOn) {
    			led.ledColor = new LEDColor(64, 0, 0);
                led.writeColor();	
    		} else {
    			led.ledColor = new LEDColor(255, 0, 32);
                led.writeColor();
    		}
    		//ledData.set(dataOn);
    	}
    	previousButtonData = rightstick.getRawButton(1);
    	
    	if (leftstick.getRawButton(3)){
    		myDrive.arcadeDrive(leftstick);
    	}
    	else{
    		myDrive.tankDrive(leftstick, rightstick);
    	}
    	
    	// TODO: TEST CODE REMOVE BEFORE COMPITITON
    	if (leftstick.getRawButton(5))
    	{
    		if (! armController.isEnabled())
    		{
    			armController.enable();    		
        		armController.setSetpoint(2.325);	
    		}
    	}
    	else
    	{
    		if (armController.isEnabled()){
    			armController.disable();
    		}
    	}
    	
    	
    	OperatorController.startOperator(this);
    	if (leftstick.getRawButton(1)){
    		if (!shiftPressed){
    			shiftToggle = !shiftToggle;
    		}
    		shiftPressed = true;
    	}
    	else{
    		shiftPressed = false;
    	}
    	if(shiftToggle){
    		shifter.set(DoubleSolenoid.Value.kForward);
    	}
    	else{
    		shifter.set(DoubleSolenoid.Value.kReverse);
    	}
    	
    	updateDashboard();
    }

    
    public void disabledPeriodic()
    {
    	updateDashboard();
    }
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    	updateDashboard();
    }
    
    public void updateDashboard() {
    	
    	// Output Encoders
    	//
    	SmartDashboard.putNumber("Right Encoder", rightDriveEncoder.getRaw());
    	SmartDashboard.putNumber("Left Encoder", leftDriveEncoder.getRaw());

    	SmartDashboard.putNumber("Arm Angle", armAngle.getAverageVoltage());
    	
    	SmartDashboard.putNumber("Gyro", mxp.getAngle());

    	//SmartDashboard.putBoolean("LED1", dataOn);
    	//SmartDashboard.putBoolean("LED2", clockOn);
    	
    	SmartDashboard.putNumber("Arm Motor", armMotor.get());
    	SmartDashboard.putNumber("Arm Ctrl", armController.get());
    	SmartDashboard.putNumber("Arm Error", armController.getAvgError());
    	SmartDashboard.putString("Arm Type", armAngle.getPIDSourceType().toString());
    }
    
}
