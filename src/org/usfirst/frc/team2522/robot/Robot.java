
package org.usfirst.frc.team2522.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
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
	// Joysticks
	// -------------------------------------------------------------------
	Joystick leftstick;
	Joystick rightstick;
	Joystick operatorstick;
	
	LEDController leds;
	Talon canBack;

	DigitalOutput do0 = new DigitalOutput(0);
	DigitalOutput do1 = new DigitalOutput(1);
	
	AnalogInput ai1 = new AnalogInput(0);
	
	AHRS mxp;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//leds = new LEDController(new DigitalOutput(16), new DigitalOutput(10), new DigitalOutput(11));
    	//leds = new LEDController(new DigitalOutput(16), new DigitalOutput(18), new DigitalOutput(19));
    	//do0.set(true);
    	//do1.set(true);
    	//leds.setColor(LEDUtil.Color.WHITE);
    	canBack = new Talon(7);
    	// Init Joysticks
		// ---------------------------------------------------------------
		leftstick = new Joystick(1);
		rightstick = new Joystick(2);
		operatorstick = new Joystick(0);
		try {
			mxp = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException e) {
			DriverStation.reportError(e.getMessage(), false);
		}
		System.out.println("Robot init");
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
      canBack.set(operatorstick.getX());
      SmartDashboard.putNumber("Pot Sensor", ai1.getValue());
      SmartDashboard.putNumber("Gyro", mxp.getAngle());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
